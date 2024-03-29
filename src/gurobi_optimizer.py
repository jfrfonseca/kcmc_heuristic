"""
Gurobi Optimizer Driver Script
"""


# STDLib
import os
import json
import argparse

# PIP
import boto3

# Payload packages
from kcmc_instance import KCMC_Instance
from gurobi_models import gurobi_multi_flow, gurobi_single_flow


def get_all_s3_objects(s3, **base_kwargs):
    continuation_token = None
    while True:
        list_kwargs = dict(MaxKeys=1000, **base_kwargs)
        if continuation_token:
            list_kwargs['ContinuationToken'] = continuation_token
        response = s3.list_objects_v2(**list_kwargs)
        yield from response.get('Contents', [])
        if not response.get('IsTruncated'):  # At the end of the list?
            break
        continuation_token = response.get('NextContinuationToken')


MODELS = {
    # 'gurobi_y_binary_single_flow': (gurobi_single_flow, True, None),
    # 'gurobi_y_binary_multi_flow': (gurobi_multi_flow, True, None),
    # 'gurobi_single_flow': (gurobi_single_flow, False, None),
    # 'gurobi_multi_flow': (gurobi_multi_flow, False, None),
    # 'dinic': (None, False, 'dinic'),
    # 'dinic_gurobi_single_flow': (gurobi_single_flow, False, 'dinic'),
    # 'dinic_gurobi_multi_flow': (gurobi_multi_flow, False, 'dinic'),
    # 'min_flood',
    # 'min_flood_gurobi_single_flow': (gurobi_single_flow, False, 'min_flood'),
    # 'min_flood_gurobi_multi_flow': (gurobi_multi_flow, False, 'min_flood'),
    # 'max_flood',
    # 'max_flood_gurobi_single_flow': (gurobi_single_flow, False, 'max_flood'),
    # 'max_flood_gurobi_multi_flow': (gurobi_multi_flow, False, 'max_flood'),
    # 'reuse',
    'no_reuse_gurobi_single_flow': (gurobi_single_flow, False, 'no_reuse'),
    'no_reuse_gurobi_multi_flow': (gurobi_multi_flow, False, 'no_reuse'),
    'min_reuse_gurobi_single_flow': (gurobi_single_flow, False, 'min_reuse'),
    'min_reuse_gurobi_multi_flow': (gurobi_multi_flow, False, 'min_reuse'),
    'max_reuse_gurobi_single_flow': (gurobi_single_flow, False, 'max_reuse'),
    'max_reuse_gurobi_multi_flow': (gurobi_multi_flow, False, 'max_reuse'),
    'best_reuse_gurobi_single_flow': (gurobi_single_flow, False, 'best_reuse'),
    'best_reuse_gurobi_multi_flow': (gurobi_multi_flow, False, 'best_reuse'),
}


def reset_process_queue(instances_file:str, models_list:list, s3_client):
    process_queue = []

    # For each instance in the file
    with open(instances_file, 'r') as fin:
        for line in fin:

            # Load the key, K and M of the instance
            key = (';'.join(line.split(';', 4)[:-1]) + ';END').upper()
            kcmc_k = int(line.split('|')[-1].split('K')[-1].split('M')[0])
            kcmc_m = int(int(line.split('|')[-1].split('M')[-1].split(')')[0]))

            # For each model, add a line to the process queue
            for model in models_list:
                process_queue.append((key, kcmc_k, kcmc_m, model))

    # Find the already-processed results in the results store
    existing_results = []
    if s3_client is None:  # LOCAL FOLDER
        for result in os.listdir(results_store):
            if not result.endswith('.json'): continue
            key, kcmc_k, kcmc_m, model_name, _ = result.lower().split('.')
            kcmc_k, kcmc_m = map(int, [kcmc_k, kcmc_m])
            key = key.split('_')
            key = f'KCMC;{key[1]} {key[2]} {key[3]};{key[4]} {key[5]} {key[6]};{key[7]};END'
            existing_results.append((key, kcmc_k, kcmc_m, model_name))
    else:
        for item in get_all_s3_objects(s3_client, Bucket=bucket_name, Prefix=s3_path):
            key, kcmc_k, kcmc_m, model_name, _ = item['Key'].lower().rsplit('/', 1)[-1].split('.')
            kcmc_k, kcmc_m = map(int, [kcmc_k, kcmc_m])
            key = key.split('_')
            key = f'KCMC;{key[1]} {key[2]} {key[3]};{key[4]} {key[5]} {key[6]};{key[7]};END'
            existing_results.append((key, kcmc_k, kcmc_m, model_name))

    with open('/data/done.json', 'r') as fin:
        done = json.load(fin)
    existing_results += [tuple(i) for i in done]

    # Find the resulting queue
    process_queue = sorted(list(set(process_queue) - set(existing_results)))
    print(f'PROCESS QUEUE HAS {len(process_queue)} ({len(existing_results)} already done)')
    return process_queue


# ######################################################################################################################
# RUNTIME


if __name__ == '__main__':

    # Set the arguments
    parser = argparse.ArgumentParser()
    parser.add_argument('-m', '--models', nargs='*', help='List of zero or more models to run. If none is specified, all models and preprocessors will be run')
    parser.add_argument('--instances_file', default='/data/instances.csv', help='Local source of instances. Default: /data/instances.csv')
    parser.add_argument('-s', '--shard', type=float, default=1.0, help='Number of Shards dot Current Offset (like a float). Defaults to 1.0. Will be overwritten by env var SHARDING')
    parser.add_argument('-l', '--limit', type=int, help='Time limit to run GUROBI, in seconds. Default: 3600', default=3600)
    parser.add_argument('-r', '--results', default='/results', help='Results store. May be an AWS S3 prefix. Defaults to /results')
    args, unknown_args = parser.parse_known_args()

    # Parse the simplest arguments
    threads = 1
    time_limit = float(str(args.limit))
    assert time_limit > 1, 'INVALID TIME LIMIT: '+str(time_limit)
    instances_file = args.instances_file.strip()
    assert os.path.isfile(instances_file), 'INVALID INSTANCES FILE: '+instances_file
    sharding = os.environ.get('SHARDING', args.shard)
    shard_total, shard_offset = map(int, str(float(sharding)).split('.'))

    # Parse the models
    if args.models is None: models = list(MODELS.keys())
    elif len(args.models) == 0: models = list(MODELS.keys())
    else: models = [m.strip().lower() for m in args.models]
    unknown_models = set(models).difference(set(MODELS.keys()))
    assert len(unknown_models) == 0, 'UNKNOWN MODELS: '+', '.join(sorted(list(unknown_models)))

    # Parse the results store
    results_store = args.results.strip()

    # If the results store is an AWS S3 prefix, start an AWS S3 client
    if results_store.startswith('s3://'):
        s3_client = boto3.client('s3')
        _, _, bucket_name, s3_path = results_store.split('/', 3)
    else:
        s3_client = None

    # Prepare the process queue
    process_queue = reset_process_queue(instances_file, models, s3_client)

    # RUNTIME ----------------------------------------------------------------------------------------------------------

    # For each object to process
    for key, kcmc_k, kcmc_m, model_name in process_queue[shard_offset::shard_total]:

        # Notify the processing of the instance
        print(f'({sharding}) PROCESSING {key} | {kcmc_k} | {kcmc_m} | {model_name}')

        # Load the instance as a python object
        instance = KCMC_Instance(key, False, True, False)

        # Get the arguments of the model
        model_factory, y_binary, prep_stage = MODELS[model_name]

        # If the model has a preprocessing stage, preprocess it
        if prep_stage:
            try:
                new_instance = instance.preprocess(kcmc_k, kcmc_m, prep_stage, raw=False, fail_if_invalid=True)
            except KeyError as kerr:
                print(f'\tINVALID PREPROCESSING {prep_stage} ON INSTANCE {key}')
                continue
            preprocessing = instance._prep.copy()

            # Notify the PRE processing of the instance
            print(f'\tPREPROCESSING {prep_stage}'
                  f' | {len(new_instance.sensors)}'
                  f' | {round(len(new_instance.inactive_sensors)*100.0 / len(instance.sensors), 3)}%')

            # Update the referred instance
            instance = new_instance

        else:
            prep_stage = ''
            preprocessing = {}

        if model_factory:

            # Build the model and its variables
            model, X, Y = model_factory(kcmc_k, kcmc_m, instance,
                                        time_limit=time_limit, threads=threads,
                                        LOGFILE='/tmp/gurobi.log', y_binary=y_binary)

            # Run the model
            results = model.optimize(compress_variables=True)

            # Get the LOGs
            with open('/tmp/gurobi.log', 'r') as fin:
                gurobi_logs = '\n'.join([l.replace('\n', '') for l in fin.readlines() if len(l.strip()) > 0])
            os.unlink('/tmp/gurobi.log')

            # Update the results with some more metadata
            results.update({
                'gurobi_y_binary': y_binary,
                'gurobi_model': model_name,
                'gurobi_logs': gurobi_logs
            })

        # If no GUROBI stage:
        else:
            results = {'gurobi_model': None, 'gurobi_logs': None, 'gurobi_y_binary': None}

        # Enrich the results
        results.update({
            'key': key, 'kcmc_k': kcmc_k, 'kcmc_m': kcmc_m,
            'pois': len(instance.pois), 'sensors': len(instance.sensors), 'sinks': len(instance.sinks),
            'coverage_radius': instance.sensor_coverage_radius,
            'communication_radius': instance.sensor_communication_radius,
            'random_seed': instance.random_seed,
            'model': model_name, 'prep_stage': prep_stage,
            'time_limit': time_limit, 'threads': threads,
            'coverage_density': instance.coverage_density,
            'communication_density': instance.communication_density,
            'preprocessing': preprocessing
        })

        # Store the results
        results_file = f'{key}.{kcmc_k}.{kcmc_m}.{model_name}'.replace(';', '_').replace(' ', '_')
        if s3_client is None:
            results_file = os.path.join(results_store, f'{results_file}.json')
            with open(results_file, 'w') as fout:
                json.dump(results, fout)
        else:
            results_file = results_file+'.json'
            with open('/tmp/'+results_file, 'w') as fout:
                json.dump(results, fout)
            s3_client.upload_file('/tmp/'+results_file, bucket_name, os.path.join(s3_path, results_file))

        print(f'DONE {results["objective_value"]}')

    print('DONE WITH A RUN OF THE QUEUE!')
