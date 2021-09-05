
import sys
import csv
import time
import traceback
import asyncio
import aioredis

LOCK_TIME = 10000
BLOCK_SIZE = 20
HEAT_UP_TIME = 30


async def generate_instances(
        random_seeds:list, k_range: int, m_range: int,
        num_pois:int, num_sensors:int, num_sinks:int, area_side:int, covg_radius:int, comm_radius:int,
        target_instances: int
) -> (int, int):
    redis = await aioredis.from_url("redis://host.docker.internal")  # Get a connection with REDIS

    # Check how many instances we already have
    instance_key = f'INSTANCE:{num_pois}:{num_sensors}:{num_sinks}:{area_side}:{covg_radius}:{comm_radius}'
    evaluation_key = f'EVALUATION:{num_pois}:{num_sensors}:{num_sinks}:{area_side}:{covg_radius}:{comm_radius}'
    existing_instances = int(await redis.hlen(instance_key))
    if existing_instances >= target_instances: return existing_instances, 0

    # If we got here, ~someone~ should create those missing instances in one of the blocks. Try to acquire the lock
    block = 0
    lock = None
    for block in range(0, target_instances, BLOCK_SIZE):
        async with redis.pipeline(transaction=True) as pipe:
            lock = f'LOCK:BLOCK{block}:{instance_key}'
            has_lock, _ = await (pipe
                .setnx(lock, str(time.time_ns()))
                .expire(lock, LOCK_TIME)
                .execute()
            )
        if has_lock: break      # Lets do this block!
        else: time.sleep(0.05)  # Prevent REDIS from overheating

    # If no lock at all, this configuration is DONE
    if not has_lock: return existing_instances, -1
    print('GOT LOCK ::', instance_key, '::>', block)

    # List the instances to generate
    to_generate = [
        rseed for rseed in random_seeds[max(block, existing_instances):target_instances]
        if (not (await redis.hexists(instance_key, rseed)))
    ]
    if len(to_generate) == 0: return existing_instances, -2

    # Run the command in a subprocess
    proc = await asyncio.create_subprocess_exec(
        '/app/instance_generator',
        *list(map(str, [k_range, m_range, num_pois, num_sensors, num_sinks, area_side, covg_radius, comm_radius]
                       + to_generate)),
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
        limit = 1024 * 512,  # 512 KiB
    )

    # Read the expected number of lines, each being piped to REDIS
    for num, expected_seed in enumerate(to_generate):

        # Instance, serialized
        out_instance = await proc.stdout.readline()
        out_instance = out_instance.decode('ascii').strip()

        # Instance evaluation
        out_evaluation = await proc.stdout.readline()
        out_evaluation = out_evaluation.decode('ascii').strip()

        # Set the data on REDIS
        async with redis.pipeline(transaction=True) as pipe:
            await (pipe
               .hset(instance_key,   str(expected_seed), out_instance)
               .hset(evaluation_key, str(expected_seed), out_evaluation)
               .execute()
            )

    # Wait for the subprocess exit
    await proc.wait()

    # Return the amount of work performed and delete the lock
    result = (int(await redis.hlen(instance_key)), len(to_generate))
    await redis.delete(lock)
    await redis.close()
    return result


if __name__ == '__main__':
    try:
        time.sleep(HEAT_UP_TIME)

        # parse the arguments
        configs_file = sys.argv[1]
        target_instances = int(sys.argv[2])
        k_range = int(sys.argv[3])
        m_range = int(sys.argv[4])

        # Extract the complete random seeds
        with open('/data/random_seeds.txt', 'r') as fin:
            random_seeds = fin.read()
        random_seeds = list(set([int(i) for i in random_seeds.replace("\t", " ").replace("\n", " ").split(" ") if len(i) > 0]))

        # Extract the configurations
        configs_to_run = []
        with open(configs_file, 'r') as fin:
            csv_fin = csv.reader(fin, delimiter=',')
            for i,line in enumerate(csv_fin):
                if i == 0: continue
                configs_to_run.append(tuple(list(map(int, map(lambda i: i.strip(), line)))))

        # For each configuration, generate instances
        run_again = True
        while run_again:
            run_again = False
            for num_pois, num_sensors, num_sinks, area_side, covg_radius, comm_radius in configs_to_run:
                try:
                    previous_work, current_work = asyncio.run(
                        generate_instances(
                            random_seeds, k_range, m_range,
                            num_pois, num_sensors, num_sinks, area_side, covg_radius, comm_radius,
                            target_instances
                        )
                    )
                    print(f'>>>> KCMC:{num_pois}:{num_sensors}:{num_sinks}:{area_side}:{covg_radius}:{comm_radius}', previous_work, current_work)
                    run_again = run_again or (current_work != 0)
                except Exception as exp:
                    if {'connection', 'reset', 'peer'}.issubset(str(exp).lower().strip()):
                        time.sleep(10)
                    else: raise exp

    except Exception as exp:
        traceback.print_exc()
