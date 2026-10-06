from pathlib import Path
import json,sys
import torch,numpy as np
root=Path('/workspace/TwoWheeledRobot');folder=root/'logs/gru_response_probe_2026-10-06'
torch.set_num_threads(1)
base=root/'logs/rsl_rl/nn_drive_fixed_stance/2026-08-10_23-51-29_range_gru_stage5/model_775.pt'
run=root/'logs/rsl_rl/nn_drive_fixed_stance/2026-10-06_07-07-54_gru_recovery_floor015_s43_2026-10-06_retry1_stage5'
paths=[base,*[run/f'model_{n}.pt' for n in sys.argv[1:] if (run/f'model_{n}.pt').exists()]]
data=json.loads((folder/'hardware_inputs.json').read_text())['rows'];results={}
for path in paths:
 checkpoint=torch.load(path,map_location='cpu',weights_only=False);state=checkpoint['model_state_dict']
 gru=torch.nn.GRU(13,64);head=torch.nn.Sequential(torch.nn.Linear(64,64),torch.nn.ReLU(),torch.nn.Linear(64,64),torch.nn.ReLU(),torch.nn.Linear(64,2))
 gru.load_state_dict({k.removeprefix('memory_a.rnn.'):v for k,v in state.items() if k.startswith('memory_a.rnn.')})
 head.load_state_dict({k.removeprefix('actor.'):v for k,v in state.items() if k.startswith('actor.')})
 h=torch.zeros(1,1,64);age=0;prev_seq=None;by_stage={};errors=[]
 with torch.inference_mode():
  for row in data:
   if prev_seq is not None and row['seq']!=prev_seq+1: h.zero_();age=0
   prev_seq=row['seq']
   if bool(row['flags']&2) and not bool(row['flags']&4): h.zero_();age=0;continue
   if row['obs'][9]==0 and row['obs'][10]==0: h.zero_();age=400
   obs=torch.tensor(row['obs'],dtype=torch.float32).reshape(1,1,13)
   feature,h=gru(obs,h);action=(2*torch.tanh(head(feature.squeeze(0)))).numpy()[0];age+=1
   if age<400: continue
   errors.append(float(np.mean(np.abs(action-np.array(row['action'])))))
   by_stage.setdefault(row['stage'],[]).append(float(np.mean(action**2)))
 name='old' if path==base else path.stem
 results[name]={'mean_absolute_difference_from_old_action_a':float(np.mean(errors)),'stage_rms_current_a':{k:float(np.sqrt(np.mean(v))) for k,v in by_stage.items()},'checkpoint':str(path),'optimizer_learning_rate':checkpoint['optimizer_state_dict']['param_groups'][0]['lr']}
 print(name,round(results[name]['mean_absolute_difference_from_old_action_a'],6),{s:round(results[name]['stage_rms_current_a'][s],4) for s in ['forward','reverse','final_hold']})
(folder/'checkpoint_replay.json').write_text(json.dumps(results,indent=2)+'\n')
assert results['old']['mean_absolute_difference_from_old_action_a']<1e-5
