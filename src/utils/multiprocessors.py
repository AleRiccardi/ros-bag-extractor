import time


def proc_start_check(p, processes):
    # Start the process
    p.start()
    processes.append(p)

    # while len(processes) > multiprocessing.cpu_count():
    while len(processes) > 4:
        # Sleep for 50 milliseconds
        time.sleep(0.01)
        # Remove the dead processes
        to_remove = []
        for idx in range(len(processes)):
            processes[idx].join(timeout=0)
            if not processes[idx].is_alive():
                to_remove.append(processes[idx])

        for process in to_remove:
            processes.remove(process)


def proc_join_all(processes):
    for process in processes:
        process.join()
