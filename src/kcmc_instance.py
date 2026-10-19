"""
KCMC_Instance Object

Run from the current dir to test the KCMC_Instance class.
> python kcmc_instance ../data/instances.csv
"""

# STDLib
from collections import defaultdict
import os, json, sys

# PIP
import pandas as pd


class KCMC_Instance():

    def __init__(
        self,
        num_pois: int, num_sensors: int, num_sinks: int,
        area_side: int, sensor_coverage_radius: int, sensor_communication_radius: int,
        random_seed: int,
        data_dir='data'
    ):
        if not data_dir.startswith(os.path.sep): data_dir = os.path.join(os.path.dirname(__file__), data_dir)

        # Parse the base attributes
        self.num_pois = int(num_pois)
        self.num_sensors = int(num_sensors)
        self.num_sinks = int(num_sinks)
        self.area_side = int(area_side)
        self.sensor_coverage_radius = int(sensor_coverage_radius)
        self.sensor_communication_radius = int(sensor_communication_radius)
        self.random_seed = int(random_seed)
        assert self.num_sinks == 1, 'THIS VERSION ONLY SUPPORTS SINGLE-SINK!'

        # Format complex attributes
        self.instance_key = f'{self.num_pois} {self.num_sensors} {self.num_sinks};{self.area_side} {self.sensor_coverage_radius} {self.sensor_communication_radius};{self.random_seed}'
        self.instance_group_key = f'{self.num_pois}_{self.num_sensors}_{self.num_sinks}_{self.area_side}_{self.sensor_coverage_radius}_{self.sensor_communication_radius}'
        self.cx2_file = os.path.join(data_dir, 'cx2', f'kcmc_instance_{self.instance_group_key}_{self.random_seed}.cx2')

        # Read the heuristic results for the instance
        self.heuristics_results_file = os.path.join(data_dir, 'heuristics_results.tsv')
        df = pd.read_csv(self.heuristics_results_file, sep='\t', header=None, names=[
            'valid', 'instance_key', 'kcmc_k', 'kcmc_m', 'applied_pre_heuristic',
            'pre_heuristic_processing_time', 'pre_heuristic_solution_size', 'pre_heuristic_solution'
        ])
        df = df[df['instance_key'] == self.instance_key].copy(deep=True)
        assert all(df['valid'] == 'OK'), f'INVALID HEURISTIC RESULTS {self.instance_key}\n{df.to_markdown()}'
        assert len(df) == 6, f'MISSING PRE HEURISTICS {len(df)} | {self.instance_key}'
        assert len(df['kcmc_k'].unique()) == 1, 'Invalid KCMC K'
        assert len(df['kcmc_m'].unique()) == 1, 'Invalid KCMC M'

        # Parse the pre-heuristic results
        self.kcmc_k = int(df.iloc[0]['kcmc_k'])
        self.kcmc_m = int(df.iloc[0]['kcmc_m'])
        self.pre_heuristic_results = {
            ph['applied_pre_heuristic']: {'processing_time': ph['pre_heuristic_processing_time'], 'solution': ph['pre_heuristic_solution']}
            for _, ph in df.iterrows()
        }

        # Read the CX2 serialization of the instance
        with open(self.cx2_file, 'r') as f:
            self.cx2 = f.read()

        # Parse the nodes
        data = json.loads(self.cx2)
        nodes = {
            n['id']: {
                'type': n['v']['node_type'],
                'name': n['v']['name'],
                'id': n['id'],
                'x': float(n['x']),
                'y': float(n['y'])
            }
            for n in data[3]['nodes']
        }
        assert len(nodes) > 0, 'NO NODES FOUND'
        assert len(nodes) == self.num_pois + self.num_sensors + self.num_sinks, 'INCORRECT NUMBER OF NODES'
        assert all(n['type'] in ['POI', 'SENSOR', 'SINK'] for n in nodes.values()), 'INVALID NODE TYPES'
        assert all((('x' in n) and ('y' in n)) for n in nodes.values()), 'NODES MISSING COORDINATES'
        assert all(isinstance(n['x'], float) and isinstance(n['y'], float) for n in nodes.values()), 'NODE COORDINATES MUST BE FLOATS'
        assert all(((0 <= n['x'] <= self.area_side) and (0 <= n['y'] <= self.area_side)) for n in nodes.values()), 'NODE COORDINATES OUT OF RANGE'

        self.pois:dict[str,dict] = {p['id']: p for p in nodes.values() if p['type'] == 'POI'}
        self.sensors:dict[str,dict] = {p['id']: p for p in nodes.values() if p['type'] == 'SENSOR'}
        self.sinks:dict[str,dict] = {p['id']: p for p in nodes.values() if p['type'] == 'SINK'}

        # Parse the edges
        self.poi_sensor:dict[str,set[str]] = defaultdict(set)
        self.sensor_poi:dict[str,set[str]] = defaultdict(set)
        self.sensor_sensor:dict[str,set[str]] = defaultdict(set)
        self.sensor_sink:dict[str,set[str]] = defaultdict(set)
        self.sink_sensor:dict[str,set[str]] = defaultdict(set)
        for edge in data[4]['edges']:
            source = nodes[edge['s']]
            target = nodes[edge['t']]

            if source['type'] == 'POI':
                assert target['type'] == 'SENSOR', 'POI MUST CONNECT TO SENSOR'
                self.poi_sensor[source['id']].add(target['id'])
                self.sensor_poi[target['id']].add(source['id'])

            elif source['type'] == 'SENSOR':
                assert target['type'] != 'POI', 'THERE MUST BE NO SENSOR-POI EDGES'

                if target['type'] == 'SINK':
                    self.sensor_sink[source['id']].add(target['id'])
                    self.sink_sensor[target['id']].add(source['id'])

                elif target['type'] == 'SENSOR':
                    self.sensor_sensor[source['id']].add(target['id'])
                    self.sensor_sensor[target['id']].add(source['id'])

                else: raise ValueError(f'INVALID EDGE FROM {source["type"]} TO {target["type"]}')

            else: raise ValueError(f'INVALID EDGE FROM {source["type"]} TO {target["type"]}')

        # Add the edges attributes and inverse coverage graph
        self.poi_edges:list[tuple[str, str]] = sorted(set([(f'p{p}', f'i{i}') for p, sensors in self.poi_sensor.items() for i in sensors]))
        self.sensor_edges:list[tuple[str, str]] = sorted(set([(f'i{i}', f'i{ii}') for i, sensors in self.sensor_sensor.items() for ii in sensors]))
        self.sink_edges:list[tuple[str, str]] = sorted(set([(f'i{i}', f's{s}') for i, sinks in self.sensor_sink.items() for s in sinks]))
        self.inverse_coverage_graph:dict[str, set[str]] = {f'p{p}': set([f'i{i}' for i in sensors]) for p, sensors in self.poi_sensor.items()}


if __name__ == "__main__":
    instances_file = sys.argv[1]
    data_dir = os.path.dirname(os.path.abspath(instances_file))

    from tqdm import tqdm
    with open(instances_file, 'r') as f:
        for line in tqdm(f):
            (pois, sensors, sinks,
             area_side, covg_r, comm_r,
             random_seed
            ) = map(int, line.split(' | ')[0].replace('KCMC;', '').replace(';END', '').replace(';', ' ').strip().split(' '))

            # Test loading the instance
            kcmc = KCMC_Instance(
                num_pois=pois,
                num_sensors=sensors,
                num_sinks=sinks,
                area_side=area_side,
                sensor_coverage_radius=covg_r,
                sensor_communication_radius=comm_r,
                random_seed=random_seed,
                data_dir=data_dir
            )

    print('ALL OK')
