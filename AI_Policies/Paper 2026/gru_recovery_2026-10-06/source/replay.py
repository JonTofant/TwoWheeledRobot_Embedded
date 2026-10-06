from pathlib import Path
import json
import numpy as np
import onnxruntime as ort
root=Path('/workspace/TwoWheeledRobot')
folder=root/'logs/gru_response_probe_2026-10-06'
data=json.loads((folder/'hardware_inputs.json').read_text())
opts=ort.SessionOptions();opts.intra_op_num_threads=1;opts.inter_op_num_threads=1
models={'old':root/'ExportedPolicy/fixedstance_range_gru_2026-08-10/2026-08-10_23-51-29_range_gru_stage5/exported/policy_drive.onnx','r2_c43':root/'ExportedPolicy/r2_hardware_s43/arm_C_range_gru_s43.onnx'}
result={'source':data['source'],'source_sha256':data['source_sha256'],'limitation':'Replay of historical old-policy sensor trajectory, not closed-loop evaluation. Initial recurrent state is unavailable; checkpoint resets are reconstructed from exact-zero previous-current features. No model strength or hardware transfer guarantee follows from replay.'}
for label,path in models.items():
 session=ort.InferenceSession(str(path),sess_options=opts,providers=['CPUExecutionProvider'])
 h=np.zeros((1,1,64),np.float32);age=0;prev_seq=None;errors=[];currents=[];by_stage={}
 for row in data['rows']:
  if prev_seq is not None and row['seq']!=prev_seq+1: h.fill(0);age=0
  prev_seq=row['seq']
  fallen=bool(row['flags'] & 2) and not bool(row['flags'] & 4)
  if fallen: h.fill(0);age=0;continue
  # The firmware zeros prev commands and hidden state together at checkpoints.
  if row['obs'][9]==0 and row['obs'][10]==0:
   h.fill(0);age=400
  obs=np.asarray([row['obs']],np.float32)
  action,h=session.run(None,{'obs':obs,'h_in':h});age+=1
  if age<400: continue
  current=float(np.sqrt(np.mean(action**2)))
  error=float(np.mean(np.abs(action[0]-np.asarray(row['action']))))
  errors.append(error);currents.append(current)
  stage=by_stage.setdefault(row['stage'],{'current':[],'error':[]})
  stage['current'].append(current);stage['error'].append(error)
 result[label]={'frames':len(errors),'mean_abs_difference_from_recorded_action_a':float(np.mean(errors)),'rms_replayed_current_a':float(np.sqrt(np.mean(np.square(currents)))),'by_stage':{k:{'frames':len(v['error']),'mean_abs_difference_from_recorded_action_a':float(np.mean(v['error'])),'rms_replayed_current_a':float(np.sqrt(np.mean(np.square(v['current']))))} for k,v in by_stage.items()}}
 print(label,result[label]['frames'],result[label]['mean_abs_difference_from_recorded_action_a'],result[label]['rms_replayed_current_a'])
(folder/'hardware_replay.json').write_text(json.dumps(result,indent=2)+'\n')
