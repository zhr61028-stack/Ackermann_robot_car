import os, subprocess, signal, time
from pathlib import Path
records=Path(__file__).resolve().parents[1]/"验证报告"
env=os.environ.copy();env["ROS_DOMAIN_ID"]="71"
# Dedicated domain prevents collisions with unrelated ROS sessions.
procs=[]
try:
    for name,args in [("talker",["ros2","run","demo_nodes_cpp","talker"]),("listener",["ros2","run","demo_nodes_py","listener"])]:
        f=(records/(name+".log")).open("w")
        procs.append((subprocess.Popen(args,env=env,stdout=f,stderr=subprocess.STDOUT,start_new_session=True),f))
    time.sleep(7)
    for name,args in [("nodes",["ros2","node","list","--no-daemon"]),("topics",["ros2","topic","list","--no-daemon"]),("chatter_info",["ros2","topic","info","/chatter","--verbose"] )]:
        r=subprocess.run(args,env=env,capture_output=True,text=True,timeout=15)
        (records/(name+".txt")).write_text(r.stdout+r.stderr)
        print(name,r.returncode,r.stdout[:1200])
finally:
    for proc,f in procs:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:os.killpg(proc.pid,signal.SIGTERM);proc.wait(timeout=5)
        f.close()
s=(records/"listener.log").read_text();ok="I heard" in s
print("COMMUNICATION_PASS",ok)
raise SystemExit(0 if ok else 1)
