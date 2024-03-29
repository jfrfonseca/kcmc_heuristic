"""
KCMC_Instance Object
"""


import json
import subprocess
import sys
from typing import List, Set, Tuple, Dict
try:
    import igraph
except Exception as exp:
    igraph = None


# Get placements using C++ interface
def get_placements(pois, sensors, sinks, area_side, random_seed, executable='/app/placements_visualizer'):
    out = subprocess.Popen(list(map(str, [executable, pois, sensors, sinks, area_side, random_seed])),
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    stdout,stderr = out.communicate()
    assert stderr is None, f'ERROR ON THE PLACEMENTS VISUALIZER.\nSTDOUT:{stdout}\n\nSTDERR:{stderr}'
    placements = {}
    for row in stdout.decode().strip().splitlines():
        row = row.strip()
        if row.startswith('id'): continue
        i, x, y = row.split(',')
        placements[i] = (int(x), int(y))
    return placements


# Get the regenerated instance from its key using the C++ interface
def get_serialized_instance(pois, sensors, sinks, area_side, sensor_coverage_radius, sensor_communication_radius, random_seed,
                            executable='/app/instance_evaluator'):
    out = subprocess.Popen(
        list(map(str, [
            executable, 0, 0,
            f'KCMC;{pois} {sensors} {sinks}; {area_side} {sensor_coverage_radius} {sensor_communication_radius}; {random_seed}; END'
        ])),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )
    stdout,stderr = out.communicate()
    assert (len(stdout) > 10) and (stderr is None), f'ERROR ON THE INSTANCE REGENERATOR.\nSTDOUT:{stdout}\n\nSTDERR:{stderr}'
    return stdout.decode().strip().splitlines()[0].strip()


# Get the regenerated instance from its key using the C++ interface
def get_preprocessing(pois, sensors, sinks, area_side, sensor_coverage_radius, sensor_communication_radius, random_seed,
                      kcmc_k, kcmc_m, executable='/app/optimizer'):

    # Run the C++ package
    instance_key = f'KCMC;{pois} {sensors} {sinks}; {area_side} {sensor_coverage_radius} {sensor_communication_radius};{random_seed};END'
    out = subprocess.Popen(
        list(map(str, [
            executable,
            instance_key,
            kcmc_k, kcmc_m
        ])),
        stdout=subprocess.PIPE, stderr=subprocess.STDOUT
    )

    # Parse the results
    stdout,stderr = out.communicate()
    assert (len(stdout) > 10) and (stderr is None), f'ERROR ON THE INSTANCE REGENERATOR.\nSTDOUT:{stdout}\n\nSTDERR:{stderr}'
    result = {}
    for line in stdout.decode().strip().splitlines():
        content = line.lower().strip().split('\t')[3:]
        assert len(content) == 6, f'INVALID LINE.\nSTDOUT:{stdout.decode()}\n\nSTDERR:{stderr}'
        item = dict(zip(['method', 'runtime_us', 'valid_result', 'num_used_sensors', 'compression_rate', 'solution'], content))

        # Normalize the item method
        if item['method'][-1].isdigit():
            item['method'], num_paths = item['method'].rsplit('_', 1)
            item['num_paths'] = int(num_paths)

        # Normalize the values and store
        item['valid_result'] = str(item['valid_result']).upper() == 'OK'
        item['runtime_us'] = int(item['runtime_us'])
        item['num_used_sensors'] = int(item['num_used_sensors'])
        item['compression_rate'] = float(item['compression_rate'])
        result[item['method']] = item.copy()
    return result


class KCMC_Instance(object):

    color_dict = {
        "p": "green",
        "i": "red",          "offline": "white",
        "s": "black",        "vsink": "grey",
        "tree_0": "blue",
        "tree_1": "orange",
        "tree_2": "yellow",
        "tree_3": "magenta",
        "tree_4": "cyan"
    }

    def __repr__(self): return f'<{self.key_str} {self.random_seed} [{len(self.virtual_sinks)}]>'

    def parse_serialized_instance(self, instance_string:str, inactive_sensors=None):

        self.string = instance_string.upper().strip()
        if inactive_sensors is None: inactive_sensors = set()

        assert self.string.startswith('KCMC;'), 'ALL INSTANCES MUST START WITH THE TAG <KCMC;>'
        assert self.string.endswith(';END'), 'ALL INSTANCES MUST END WITH THE TAG <;END>'

        # IDENTIFY AND UPGRADE STRING VERSION
        self.version = 0.1 if ';SK;' in self.string else 1.0
        if self.version == 0.1: self.string = self.string.replace('PS', 'PI').replace('SS', 'II').replace('SK', 'IS')

        # Base-parse the string
        instance = self.string.split(';')

        # Parse the constants
        try:
            self.num_pois, self.num_sensors, self.num_sinks = instance[1].split(' ')
            self.area_side, self.sensor_coverage_radius, self.sensor_communication_radius = instance[2].split(' ')
            self.random_seed = instance[3]
            self.num_pois, self.num_sensors, self.num_sinks, self.random_seed \
                = map(int, [self.num_pois, self.num_sensors, self.num_sinks, self.random_seed])
            self.area_side, self.sensor_coverage_radius, self.sensor_communication_radius \
                = map(int, [self.area_side, self.sensor_coverage_radius, self.sensor_communication_radius])
        except Exception as exp: raise AssertionError('INVALID INSTANCE PREAMBLE!')

        # Prepare the buffers
        self.inactive_sensors = inactive_sensors
        self._prep = None
        self._placements = None
        self.poi_sensor = {}
        self.sensor_poi = {}
        self.sensor_sensor = {}
        self.sink_sensor = {}
        self.sensor_sink = {}
        self.edges = {}
        self.virtual_sinks_map = {}

        tag = None
        is_expanded = False
        for i, token in enumerate(instance[4:-1]):
            if token in {'PI', 'II', 'IS'}:
                tag = token
                continue
            elif tag is None:
                raise AssertionError(f'INVALID TAG PARSING AT TOKEN {i+4} - {token}')

            alpha, beta = map(int, token.strip().split(' '))
            if tag == 'PI':
                if f'i{beta}' in inactive_sensors: continue  # Skip inactive sensors
                is_expanded = True
                self._add_to(self.edges, f'p{alpha}', f'i{beta}')
                self._add_to(self.poi_sensor, alpha, beta)
                self._add_to(self.sensor_poi, beta, alpha)
            elif tag == 'II':
                assert alpha != beta,  "SELF-DIRECTED EDGES ARE NOT SUPPORTED"
                if len({f'i{alpha}', f'i{beta}'}.intersection(inactive_sensors)) > 0: continue  # Skip inactive sensors
                is_expanded = True
                self._add_to(self.edges, f'i{alpha}', f'i{beta}')
                self._add_to(self.sensor_sensor, alpha, beta)
                self._add_to(self.sensor_sensor, beta, alpha)
            elif tag == 'IS':
                if f'i{alpha}' in inactive_sensors: continue  # Skip inactive sensors
                is_expanded = True
                self._add_to(self.edges, f'i{alpha}', f's{beta}')
                self._add_to(self.sensor_sink, alpha, beta)
                self._add_to(self.sink_sensor, beta, alpha)
            else: raise AssertionError(f'IMPOSSIBLE TAG {tag}')

        # If not expanded, regenerate the instance and re-parse
        if not is_expanded:
            self.parse_serialized_instance(get_serialized_instance(
                self.num_pois, self.num_sensors, self.num_sinks,
                self.area_side, self.sensor_coverage_radius, self.sensor_communication_radius,
                self.random_seed
            ), inactive_sensors)

    def __init__(self, instance_string:str,
                 accept_loose_pois=False,
                 accept_loose_sensors=False,
                 accept_loose_sinks=False,
                 inactive_sensors=None):
        self.acceptance = (accept_loose_pois, accept_loose_sensors, accept_loose_sinks)

        # Parse the serialized instance
        self.parse_serialized_instance(instance_string, inactive_sensors)

        # Reading validations
        assert max(self.poi_sensor.keys()) < self.num_pois, f'INVALID POI IDs! {[p for p in self.poi_sensor.keys() if p > self.num_pois]}'
        assert max(self.sensor_sensor.keys()) < self.num_sensors, f'INVALID SENSOR IDs! {[p for p in self.sensor_sensor.keys() if p > self.num_sensors]}'
        assert max(self.sink_sensor.keys()) < self.num_sinks, f'INVALID SINK IDs! {[p for p in self.sink_sensor.keys() if p > self.num_sinks]}'

        # Optional validations
        assert ((len(self.poi_sensor) == self.num_pois) or accept_loose_pois), \
            f'INVALID NUMBER OF POIS ({self.num_pois} {len(self.poi_sensor)})'
        assert ((len(self.sink_sensor) == self.num_sinks) or accept_loose_sinks), \
            f'INVALID NUMBER OF SINKS ({self.num_sinks} {len(self.sink_sensor)})'
        assert ((len(self.sensor_sensor) == self.num_sensors) or accept_loose_sensors), \
            f'INVALID NUMBER OF SENSORS ({self.num_sensors} {len(self.sensor_sensor)} {[i for i in range(self.num_sensors) if i not in self.sensor_sensor]})'

    # BASIC PROPERTIES #################################################################################################

    @property
    def key(self) -> tuple:
        return self.num_pois, self.num_sensors, self.num_sinks, \
               self.area_side, self.sensor_coverage_radius, self.sensor_communication_radius

    @property
    def key_str(self) -> str:
        p,i,s, a,cv,cm, = self.key
        return f'KCMC;{p} {i} {s};{a} {cv} {cm};{self.random_seed};END'

    @property
    def is_single_sink(self) -> bool: return self.num_sinks == 1

    @property
    def pois(self) -> List[str]: return [f'p{p}' for p in self.poi_sensor.keys()]

    @property
    def poi_degree(self) -> Dict[str, int]: return {f'p{p}': len(i) for p, i in self.poi_sensor.items()}

    @property
    def poi_edges(self) -> List[Tuple[str, str]]: return [(f'p{p}', f'i{i}') for p, sensors in self.poi_sensor.items() for i in sensors]

    @property
    def sensors(self) -> List[str]: return sorted(list(self.sensor_degree.keys()))

    @property
    def sensor_degree(self) -> Dict[str, int]: return {f'i{p}': len(i) for p, i in self.sensor_sensor.items()}

    @property
    def original_sensors(self) -> Set[str]: return set([s for s in self.sensors if s not in self.virtual_sinks])

    @property
    def sensor_edges(self) -> List[Tuple[str, str]]: return [(f'i{i}', f'i{ii}') for i, sensors in self.sensor_sensor.items() for ii in sensors]

    @property
    def sinks(self) -> List[str]: return [f's{k}' for k in self.sink_sensor]

    @property
    def sink_degree(self) -> Dict[str, int]: return {f's{p}': len(i) for p, i in self.sink_sensor.items()}

    @property
    def sink_edges(self) -> List[Tuple[str, str]]: return [(f'i{i}', f's{s}') for i, sinks in self.sensor_sink.items() for s in sinks]

    @property
    def coverage_density(self) -> float: return (self.sensor_coverage_radius*self.num_sensors)/(self.area_side*self.area_side*self.num_pois)

    @property
    def communication_density(self) -> float: return (self.sensor_communication_radius*self.num_sensors*self.num_sinks)/(self.area_side*self.area_side)

    @property
    def virtual_sinks(self) -> Set[str]: return set([f'i{s}' for vsinks in self.virtual_sinks_map.values() for s in vsinks])

    @property
    def original_sinks(self) -> Set[str]: return set([f'i{s}' for s in self.virtual_sinks_map.keys()])

    @property
    def virtual_sinks_dict(self) -> Dict[str, str]:
        inverted_virtual_sinks_map = {}
        for osink, virtual_sinks in self.virtual_sinks_map.items():
            for vsink in virtual_sinks:
                inverted_virtual_sinks_map[f'i{vsink}'] = f's{osink}'
        return inverted_virtual_sinks_map

    @property
    def num_virtual_sinks(self) -> bool: return len(self.virtual_sinks)

    @property
    def dual_edges(self):
        return sorted(list(set(
            [(a, b) for a, l in self.edges.items() for b in l if (a != b)]
          + [(b, a) for a, l in self.edges.items() for b in l if (a != b)]
        )))

    @property
    def linear_edges(self):
        edges = set()
        for a, b in self.dual_edges:
            if (a, b) not in edges:
                if (b, a) not in edges:
                    edges.add((a, b))
        return sorted(list(edges))

    @property
    def coverage_graph(self) -> Dict[str, Set[str]]: return {f'i{i}': set([f'p{p}' for p in pois]) for i, pois in self.sensor_poi.items()}

    @property
    def inverse_coverage_graph(self) -> Dict[str, Set[str]]: return {f'p{p}': set([f'i{i}' for i in sensors]) for p, sensors in self.poi_sensor.items()}

    @property
    def communication_graph(self) -> Dict[str, Set[str]]: return {f'i{i}': set([f'i{p}' for p in sensors]) for i, sensors in self.sensor_sensor.items()}

    @property
    def placements(self):
        if self._placements is None:
            self._placements = get_placements(self.num_pois, self.num_sensors, self.num_sinks, self.area_side, self.random_seed)
        return self._placements.copy()

    # SERVICES #########################################################################################################

    def preprocess(self, k, m, prep_method:str, raw=True, fail_if_invalid=True):

        # Memoize the preprocessing
        if self._prep is None:
            self._prep = get_preprocessing(
                self.num_pois, self.num_sensors, self.num_sinks,
                self.area_side, self.sensor_coverage_radius, self.sensor_communication_radius,
                self.random_seed, k, m
            )

        # If we have a valid preprocessing:
        if (len(self._prep) > 0) and self._prep.get(prep_method, {}).get('valid_result', False):

            # Return the raw result if required
            if raw: return self._prep[prep_method]

            # Parse the raw result as a new instance
            inactive_sensors = {f'i{j}' for j,i in enumerate(self._prep[prep_method]['solution']) if i == '0'}
            return KCMC_Instance(self.key_str, *self.acceptance, inactive_sensors=inactive_sensors)

        # If invalid preprocessing
        else:
            if fail_if_invalid: raise KeyError(f'Invalid preprocessing method on instance {self.key_str}: {prep_method}')
            if raw: return {
                'method': prep_method,
                'runtime_us': 1_000_000,
                'valid_result': False,
                'num_used_sensors': len(self.sensors),
                'compression_rate': 0.0,
                'solution': '1'*len(self.sensors)
            }
            return self

    @staticmethod
    def _add_to(_dict:dict, key, value):
        if key not in _dict:
            _dict[key] = set()
        _dict[key].add(value)

    def to_single_sink(self, MAX_M=10):
        if self.is_single_sink: return self

        # For each sink and corresponding vsink
        vsinks_added = []
        virtual_sinks_map = {}
        new_sensor_sensor = {s: ls.copy() for s, ls in self.sensor_sensor.items()}
        for sink, sensors in self.sink_sensor.items():
            for vsink in range(self.num_sensors+(sink*MAX_M), self.num_sensors+((sink+1)*MAX_M)):
                vsinks_added.append(vsink)
                self._add_to(virtual_sinks_map, sink, vsink)
                for i in sensors:
                    self._add_to(new_sensor_sensor, vsink, i)
                    self._add_to(new_sensor_sensor, i, vsink)

                # Start a new instance with an modified string
        result = KCMC_Instance(
            instance_string=';'.join([
                'KCMC',
                f'{self.num_pois} {self.num_sensors+(MAX_M*self.num_sinks)} 1',
                f'{self.area_side} {self.sensor_coverage_radius} {self.sensor_communication_radius}',
                f'{self.random_seed}'
             ] +['PI']+[f'{p} {i}' for p, sensors in self.poi_sensor.items() for i in sensors]
               +['II']+[f'{p} {i}' for p, sensors in new_sensor_sensor.items() for i in sensors]
               +['IS']+[f'{vs} 0' for vs in vsinks_added]
               +['END']),
            accept_loose_pois=True, accept_loose_sensors=True, accept_loose_sinks=True
        )
        result.virtual_sinks_map = virtual_sinks_map
        return result

    # PLOTTING IN CYTOSCAPE.JS #########################################################################################

    @staticmethod
    def cytoscape_node(_id:str, name=None, weight=None, color=None, classes=None, x=None, y=None):

        # Model the data
        data = {"id": str(_id), "name": "node"+str(_id) if name is None else str(name)}
        if weight is not None: data['score'] = float(weight)

        # Model the result
        result = {
            "data": data, "group": "nodes", "classes": list(map(str, [] if classes is None else classes)),
            "locked": False, "removed": False, "selected": False, "selectable": True, "grabbable": True,
            "position": {"x": float(x), "y": float(y)} if None not in [x, y] else {}
        }

        # Add color (optional)
        if color is not None:
            if 'style' not in result: result['style'] = {}
            result['style']['background-color'] = str(color)

        return result

    @staticmethod
    def cytoscape_edge(_id:str, source:str, target:str, weight=None, width=None, color=None):

        # Model the data
        data = {"id": str(_id), "source": str(source), "target": str(target)}
        if weight is not None: data['weight'] = float(weight)

        # Model the result
        result = {
            "data": data, "group": "edges", "classes": "", "position": {},
            "locked": False, "removed": False, "selected": False, "selectable": True, "grabbable": True
        }

        # Add edge width (optional)
        if width is not None:
            if 'style' not in result: result['style'] = {}
            result['style']['width'] = str(int(width))+'px'

        # Add color (optional)
        if color is not None:
            if 'style' not in result: result['style'] = {}
            result['style']['lineColor'] = str(color)

        return result

    def cytoscape_iter(self):
        # Iteration method, to allow for different types of usages

        # Get the placements
        placements = self.placements

        # Add all POIs, sensors and sinks
        for p in self.pois:
            yield self.cytoscape_node(
                _id=p, name=p,
                x=placements[p][0], y=placements[p][1],
                weight=0.01, color=self.color_dict['p']
            )
        for i in self.sensors:
            yield self.cytoscape_node(
                _id=i, name=i,
                x=placements[i][0], y=placements[i][1],
                weight=0.0033, color=self.color_dict['i']
            )
        for s in self.sinks:
            yield self.cytoscape_node(
                _id=s, name=s,
                x=placements[s][0], y=placements[s][1],
                weight=0.02, color=self.color_dict['s']
            )

        # Add all poi-sensor edges
        for p, n_sensors in self.poi_sensor.items():
            p = f'p{p}'
            for i in n_sensors:
                i = f'i{i}'
                yield self.cytoscape_edge(_id=p+i, source=p, target=i, width=3, color='green')

        # Add all sensor-sensor edges
        for ss, n_sensors in self.sensor_sensor.items():
            ss = f'i{ss}'
            for st in n_sensors:
                st = f'i{st}'
                if int(ss[1:]) >= int(st[1:]): continue  # Avoid both back-edges and self-edges (directed graph)
                yield self.cytoscape_edge(_id=ss+st, source=ss, target=st, width=6)

        # Add all sensor-sink edges
        for i, n_sinks in self.sensor_sink.items():
            i = f'i{i}'
            for s in n_sinks:
                s = f's{s}'
                yield self.cytoscape_edge(_id=i+s, source=i, target=s, width=3, color='red')

    def cytoscape(self, target_file=None):
        # If no file is provided, return the (potentially very large!) list of dictionaries
        if target_file is None: return list(self.cytoscape_iter())

        # Write directly to a file, iteractively
        num_rows = 0
        with open(target_file, 'w') as fout:
            fout.write('[')
            has_previous = False
            for line in self.cytoscape_iter():
                if has_previous: fout.write(',')
                fout.write('\n\t')
                fout.write(json.dumps(line))
                has_previous = True
                num_rows += 1
            fout.write('\n]')

        # Returns the number of data rows in the file
        return num_rows

    # PLOTTING IN iGRAPH ###############################################################################################

    def get_node_label(self, node, installation=None):
        result = 'V'+node if node in self.virtual_sinks else node
        if installation is not None:
            if node.startswith('i'):
                tree = installation.get(node)
                if tree is not None:
                    result = result + f'.{tree}'
        return result

    def get_node_color(self, node, installation=None):
        result = self.color_dict["vsink"] if node in self.virtual_sinks else self.color_dict[node[0]]
        if installation is not None:
            if node.startswith('p'): return self.color_dict['p']
            if node.startswith('s'): return self.color_dict['s']
            tree = installation.get(node)
            if tree is None:
                result = self.color_dict['offline']
            else:
                result = self.color_dict[f'tree_{int(tree) % 5}']
        return result

    def plot(self, labels=False, installation=None, minimal=False):
        assert igraph is not None, 'IGRAPH NOT INSTALLED!'
        g = igraph.Graph()
        if minimal:
            showing = set(self.pois+self.sinks)
            if installation is None:
                showing = showing.union(self.sensors)
            else:
                showing = showing.union(set([i for i, v in installation.items() if v is not None]))
            g.add_vertices(list(showing))
            g.add_edges([(i, j) for i, j in self.linear_edges if ((i in showing) and (j in showing))])
        else:
            g.add_vertices(self.pois+self.sinks+self.sensors)
            g.add_edges(self.linear_edges)
        layout = g.layout("kk")  # Kamada-Kawai Force-Directed algorithm

        # Set the COLOR of every NODE
        g.vs["color"] = [self.get_node_color(node, installation) for node in g.vs["name"]]

        # Set the NAME of the node as its label. Add the TREE it is installed on, if exists
        if labels:
            g.vs["label"] = [self.get_node_label(node, installation) for node in g.vs["name"]]
            g.vs["label_size"] = 6

        # Print as UNIDIRECTED
        g.to_undirected()
        return igraph.plot(g, layout=layout)


# ######################################################################################################################
# RUNTIME


def parse_block(df):

    # Parse the instance as a KCMC_Instance object
    df.loc[:, 'obj_instance'] = df['instance'].apply(
        lambda instance: KCMC_Instance(instance,
                                       accept_loose_pois=True,
                                       accept_loose_sensors=True,
                                       accept_loose_sinks=True)
    )

    # Extract basic attributes of the instance
    df.loc[:, 'key'] = df['obj_instance'].apply(lambda i: i.key_str)
    df.loc[:, 'random_seed'] = df['obj_instance'].apply(lambda i: i.random_seed)
    df.loc[:, 'pois'] = df['obj_instance'].apply(lambda i: i.num_pois)
    df.loc[:, 'sensors'] = df['obj_instance'].apply(lambda i: i.num_sensors)
    df.loc[:, 'sinks'] = df['obj_instance'].apply(lambda i: i.num_sinks)
    df.loc[:, 'area_side'] = df['obj_instance'].apply(lambda i: i.area_side)
    df.loc[:, 'coverage_r'] = df['obj_instance'].apply(lambda i: i.sensor_coverage_radius)
    df.loc[:, 'communication_r'] = df['obj_instance'].apply(lambda i: i.sensor_communication_radius)

    # Extract attributes of the instance that cannot be calculated from other attributes

    # Reformat the dataframe
    df = df.fillna(False).drop_duplicates(
        subset=(['key', 'random_seed'] + [col for col in df.columns if (col.startswith('K') or col.startswith('M'))])
    ).reset_index(drop=True)

    return df

if __name__ == '__main__':
    from tqdm import tqdm
    from collections import Counter
    print('Testing Isomorphism')

    # Parse the instances
    sourcefile = sys.argv[1]
    instances = []
    with open(sourcefile, 'r') as fin:
        for line in tqdm(fin):
            instances.append(KCMC_Instance(line.strip().split('|')[0].strip(), True, True, True))
    print(f'GOT {len(instances)} INSTANCES')

    # Compare instances
    for i, inst_i in enumerate(tqdm(instances)):
        for j, inst_j in enumerate(instances[i+1:]):
            j = i+1+j
            ranks_i = set(Counter([b for a, b in inst_i.poi_degree.items()]).items())
            ranks_j = set(Counter([b for a, b in inst_j.poi_degree.items()]).items())
            diff_ij = ranks_i - ranks_j
            if len(diff_ij) != 0: continue
            diff_ji = ranks_j - ranks_i
            if len(diff_ji) != 0: continue

            ranks_i = set(Counter([b for a, b in inst_i.sensor_degree.items()]).items())
            ranks_j = set(Counter([b for a, b in inst_j.sensor_degree.items()]).items())
            diff_ij = ranks_i - ranks_j
            if len(diff_ij) != 0: continue
            diff_ji = ranks_j - ranks_i
            if len(diff_ji) != 0: continue

            raise Exception(f'Possible isomorphism {i},{j}')
