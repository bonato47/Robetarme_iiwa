import pandas as pd
from scipy.spatial.transform import Rotation as R

Name = "day1_inside_spray_horizontal_dry_trial_1_0_optitrack_tf.csv"
df = pd.read_csv(Name, sep= ",")

df_reduced = pd.DataFrame()
#df_reduced["time_stamp"] = df["time_stamp"]

Pos = [0,0.5,0.5]

df_reduced["quat_x"] = df["pos_z"]
df_reduced["quat_y"] = df["pos_z"]
df_reduced["quat_z"] = df["pos_z"]
df_reduced["quat_w"] = df["pos_z"]
df_reduced["pos_x"] = df["pos_x"]  + 0.5
df_reduced["pos_y"] = df["pos_y"]
df_reduced["pos_z"] = df["pos_z"] + 0.5

df["ang_y"] = df["ang_y"]+(3.14/2)


i = 1
while i < len(df):
    r = R.from_euler('zyx', [df["ang_z"][i].astype(float), df["ang_y"][i].astype(float), df["ang_x"][i].astype(float)], degrees=False)
    q = r.as_quat()
    df_reduced["quat_x"][i]= q[0]
    df_reduced["quat_y"][i]= q[1]
    df_reduced["quat_z"][i]= q[2]
    df_reduced["quat_w"][i]= q[3]
    i = i+1

df_reduced.to_csv("trajectory.csv", index = None, header = None)
