import json, math
from pathlib import Path
import numpy as np
import onnxruntime as ort
root=Path('/workspace/TwoWheeledRobot')
models={
 'old':root/'ExportedPolicy/fixedstance_range_gru_2026-08-10/2026-08-10_23-51-29_range_gru_stage5/exported/policy_drive.onnx',
 'r2_c43':root/'ExportedPolicy/r2_hardware_s43/arm_C_range_gru_s43.onnx',
}
results={}
opt=ort.SessionOptions(); opt.intra_op_num_threads=1; opt.inter_op_num_threads=1
for label,path in models.items():
 session=ort.InferenceSession(str(path),sess_options=opt,providers=['CPUExecutionProvider'])
 rows=[]
 for target_deg in [-10,-5,-2,2,5,10]:
  h=np.zeros((1,1,64),np.float32); prev=np.zeros((1,2),np.float32)
  def step(pitch,rate):
   global h,prev
   obs=np.zeros((1,13),np.float32);obs[0,5]=1
   obs[0,2]=pitch/math.radians(25);obs[0,3]=rate/4;obs[0,9:11]=prev[0]/2
   action,h=session.run(None,{'obs':obs,'h_in':h})
   prev=np.clip(action,-2,2)
   return float(action.mean())
  for _ in range(64): step(0,0)
  rest=float(prev.mean()); commands=[]
  for t in range(10):
   angle=math.radians(target_deg)*min((t+1)/5,1)
   rate=math.radians(target_deg)/(5*.015) if t<5 else 0
   commands.append(step(angle,rate))
  rows.append({'target_pitch_deg':target_deg,'rest_current_a':rest,'common_current_a':commands,'peak_response_a':max(abs(x-rest) for x in commands)})
 results[label]=rows
out={'fixture':'64 stationary warmup steps followed by five-step pitch ramp, five-step hold; own previous command feedback; no simulated plant','limitation':'Controlled input response, not a hardware stability test','models':results}
(root/'logs/gru_response_probe_2026-10-06/results.json').write_text(json.dumps(out,indent=2)+'\n')
for label,rows in results.items(): print(label,[(r['target_pitch_deg'],round(r['peak_response_a'],3)) for r in rows])
