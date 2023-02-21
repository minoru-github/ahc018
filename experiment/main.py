#import mlflow
import pathlib
import subprocess
from subprocess import PIPE
import os
from color import *
import datetime
from multiprocessing import Pool


def prepare_execute_file():
    os.chdir('../')
    cmd = "cargo build -q --release"
    subprocess.run(cmd)
    print("build complete")

    cmd = "copy /y .\\target\\release\\ahc.exe .\\experiment\\ahc.exe"
    #cmd = "copy .\\target\\release .\\experiment\\"

    subprocess.run(cmd, shell=True)
    os.chdir('./experiment')
    path = pathlib.Path('ahc.exe')
    dt = datetime.datetime.fromtimestamp(path.stat().st_ctime)
    print("updated {}\n".format(dt.strftime('%Y年%m月%d日 %H:%M:%S')))


def parse_from_stderr(stderr: str):
    # 標準エラー出力をパース
    out = stderr.splitlines()
    cnt = int(out[0])
    score = int(out[1])
    duration = float(out[2])
    return cnt, score, duration


def run_ahc_exe(filename: pathlib.Path):
    print(filename.name)
    cmd = "ahc.exe < ./in/" + filename.name + " > ./out/" + filename.name
    path = os.path.join(os.getcwd(), filename)
    with open(path) as text:
        proc = subprocess.run(cmd, shell=True, stdin=text,
                              stdout=PIPE, stderr=PIPE, text=True, encoding="Shift-JIS")
        cnt, score, duration = parse_from_stderr(proc.stderr)
    return filename.name, cnt, score, duration


def run_multi():
    data_path = "./in/"
    input_list = []
    for filename in pathlib.Path(data_path).glob("*.txt"):
        input_list.append(filename)
    with Pool(processes=4) as p:
        result_list = p.map(func=run_ahc_exe, iterable=input_list)
    result_list.sort()
    return result_list


def output_result(result_list):
    total_score = 0
    total_cnt = 0
    for i, result in enumerate(result_list):
        filename, cnt, score, duration = result
        total_score += score
        total_cnt += cnt
        check_point_col = set_color_to_check_point(i)
        score_col = set_color_to_score(score)
        print("{} => ".format(filename[:4])
              + "total_score: {}{}{}, ".format(check_point_col,
                                               total_score, Color.RESET)
              + "score: {}{}{}, ".format(score_col, score, Color.RESET)
              + "cnt: {:5d}, ".format(cnt)
              + "total_cnt: {}{}{}, " .format(check_point_col,
                                              total_cnt, Color.RESET)
              + "time: {:.3f}".format(duration))
    print("total: {}".format(total_score))


if __name__ == "__main__":
    prepare_execute_file()
    result_list = run_multi()
    output_result(result_list)

    # total_score = compute_score()
    # with mlflow.start_run(run_name="ahc"):
    #     mlflow.log_metric(key="total score", value=total_score)
