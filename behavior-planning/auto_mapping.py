import pandas as pd
waypoints_file = '/home/zihan/avl/map/behavior-planning/area-7/saved_waypoints.csv'

df = pd.read_csv(waypoints_file)
waypoints_file_2 = '/home/zihan/avl/map/behavior-planning/area-7/saved_waypoints_1.csv'
df2 = pd.read_csv(waypoints_file_2)
df_node = pd.DataFrame(columns=['NID', 'PID'])
df_point=pd.DataFrame(columns=['PID','B','L','H','Bx','Ly','ReF','MCODE1','MCODE2','MCODE'])
df_line =pd.DataFrame(columns=['LID','BPID','FPID','BLID','FLID'])
df_idx =pd.DataFrame(columns=['Id','Kind','fname'])
df_lane = pd.DataFrame(columns=['LnID','DID','BLID','FLID','BNID','FNID','JCT','BLID2','BLID3','BLID4','FLID2','FLID3','FLID4',
                       'ClossID','Span','LCnt','Lno','LaneType','LimitVel','RefVel','RoadSecID','LaneChgFG','LinkWAID'])
index = 0
for i,row in df.iterrows():
    node_row = {'NID': index+1, 'PID': index+1}
    df_node.loc[len(df_node)] = node_row
    point_row = {'PID':index+1,'H':row['z'],'Bx':row['y'],'Ly':row['x'],'ReF':1}
    df_point.loc[len(df_point)] = point_row
    lane_row = {'LnID':index+1,'DID':0,'BLID':index,'FLID':index+2,'BNID':index+1,'FNID':index+2,'JCT':0,
                 'BLID2':0,'BLID3':0,'BLID4':0,'FLID2':0,'FLID3':0,'FLID4':0,
                       'ClossID':0,'LCnt':1,'Lno':1,'LaneType':0,
                       'LimitVel':20,'RefVel':20,'RoadSecID':0,'LaneChgFG':0,'LinkWAID':0}
    df_lane.loc[len(df_lane)] = lane_row
    index +=1
for i,row in df2.iterrows():
    node_row = {'NID': index+1, 'PID': index+1}
    df_node.loc[len(df_node)] = node_row
    point_row = {'PID':index+1,'H':row['z'],'Bx':row['y'],'Ly':row['x'],'ReF':1}
    df_point.loc[len(df_point)] = point_row
    lane_row = {'LnID':index+1,'DID':0,'BLID':index,'FLID':index+2,'BNID':index+1,'FNID':index+2,'JCT':0,
                 'BLID2':0,'BLID3':0,'BLID4':0,'FLID2':0,'FLID3':0,'FLID4':0,
                       'ClossID':0,'LCnt':1,'Lno':1,'LaneType':0,
                       'LimitVel':20,'RefVel':20,'RoadSecID':0,'LaneChgFG':0,'LinkWAID':0}
    df_lane.loc[len(df_lane)] = lane_row
    index +=1
df_node.to_csv('/home/zihan/avl/map/behavior-planning/area-7/autoware/node.csv', index=False)
df_point.to_csv('/home/zihan/avl/map/behavior-planning/area-7/autoware/point.csv', index=False)
df_line.to_csv('/home/zihan/avl/map/behavior-planning/area-7/autoware/line.csv', index=False)
df_lane.to_csv('/home/zihan/avl/map/behavior-planning/area-7/autoware/lane.csv', index=False)
df_idx.to_csv('/home/zihan/avl/map/behavior-planning/area-7/autoware/idx.csv', index=False)