import ast,json,types
from pathlib import Path
ws=Path('/home/zhanghaoran/.codex/.chatgpt-projects/Ackermann_robot_car/ackermann_sim_ws');r=ws/'验证报告/当前有效/M3_仿真收尾'
class Publisher:
 def __init__(self):self.messages=[]
 def publish(self,m):self.messages.append(m)
class Handle:
 accepted=True
 def get_result_async(self):return types.SimpleNamespace(add_done_callback=lambda f:None)
def make(path):
 module=ast.parse(path.read_text());cls=next(n for n in module.body if isinstance(n,ast.ClassDef));methods=[n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name in ['fail','follow_accepted','route_result','plan_accepted','forward']];testclass=ast.ClassDef(name='TestGenerator',bases=[],keywords=[],body=methods,decorator_list=[]);ns={'Twist':lambda:object(),'time':__import__('time')};exec(compile(ast.fix_missing_locations(ast.Module(body=[testclass],type_ignores=[])),str(path),'exec'),ns);g=ns['TestGenerator']();g.epoch=4;g.seq=7;g.plan_busy=True;g.follow=None;g.state='PLANNING';g.reason='';g.cmd=Publisher();g.get_logger=lambda:types.SimpleNamespace(error=lambda m:None);return g
future=types.SimpleNamespace(result=lambda:Handle());before=make(r/'修复前/generator.py');before.fail('ODOM_TIMEOUT');before.follow_accepted(future,4,7,False)
after=make(ws/'src/ackermann_route/ackermann_route/generator.py');after.fail('ODOM_TIMEOUT');after.follow_accepted(future,4,7,False);after.route_result(types.SimpleNamespace(result=lambda:(_ for _ in ()).throw(RuntimeError('Stale callback must not be read'))),4);after.plan_accepted(future,4,False,(0,0,0));after.last_odom=__import__('time').monotonic();after.settings={'odom_timeout_s':.5};count=len(after.cmd.messages);after.forward(object())
result={'before_late_accept_state':before.state,'after_late_accept_state':after.state,'after_stale_route_result_state':after.state,'after_late_plan_accept_state':after.state,'failed_forward_blocked':len(after.cmd.messages)==count,'epoch_after_failure':after.epoch,'pending_plan_cleared':not after.plan_busy,'all_pass':before.state=='RUNNING' and after.state=='FAILED' and len(after.cmd.messages)==count and not after.plan_busy,'test_scope':'Deterministic actual-method execution with fake action handles; not ROS lifecycle or MCU test.'};(r/'异步失败回调定向复核.json').write_text(json.dumps(result,ensure_ascii=False,indent=2));print(json.dumps(result,ensure_ascii=False,indent=2));assert result['all_pass']
