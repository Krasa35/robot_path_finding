import time
import subprocess
import argparse
import numpy as np
from datetime import datetime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-f", required=True, help="path to script beeing benchmarked")
    parser.add_argument("-i", type=int, help="optional: number of iterations (measurment runs)")
    parser.add_argument("-o", help="optional: redirect benchmarked script output to given file.")
    parser.add_argument("-t", type=float, help="optional: time limit in seconds after which script will be terminated regardless of the result.")
    parser.add_argument("script_args", nargs=argparse.REMAINDER, help="optional: list of arguments passed to benchmarked script. Precede them with --")
    args = parser.parse_args()

    path = args.f
    iters = 1 if args.i is None else args.i
    times = []
    outputFile = None
    if args.o is not None:
        outputFile = args.o
        print(f"Redirecting script output to file \"{outputFile}\"")

    print(f"Timeout set to {args.t} sec.") if args.t is not None else None

    for i in range(0, iters):
        script = ["python", path] + args.script_args[1:]
        print(f"Starting time measurment {f'run number {i+1}' if iters > 1 else ''}")
        start = time.perf_counter()
        try:
            if outputFile is None:
                subprocess.run(script, timeout=args.t, check=True)
            else:
                with open(outputFile, "a") as f:
                    f.write(f"Execution {datetime.now()}:\n")
                    f.flush()
                    subprocess.run(script,  timeout=args.t, check=True, stdout=f, stderr=subprocess.STDOUT)
        except Exception as e:
            print(f"Process finished with error: {type(e).__name__} : {e}")

        end = time.perf_counter()

        times.append(end-start)

    print("----")
    if iters == 1:
        print(f"Execution time: {times[0]:.3f} sec")
    else:
        avg = np.mean(np.array(times))
        med = np.median(np.array(times))
        run = 1
        for tim in times:
            print(f"Measurment run {run} time: {tim:.3f} sec")
            run += 1

        print(f"----\nAverage execution time: {avg:.4f} sec, median: {med:.4f} sec")

if __name__ == '__main__':
    main()
