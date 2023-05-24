import pandas as pd
from scipy.spatial.transform import Rotation as R

Name = "dataset_processed/day1_inside_spray_horizontal_wet_trial_1_0_optitrack_tf_lp.csv"
df = pd.read_csv(Name, sep= ",")

df_reduced_euler = pd.DataFrame()
#df_reduced["time_stamp"] = df["time_stamp"]

Pos = [0.5,0.5,0.5]

df_reduced_euler["ang_x"] = df["ang_x"] 
df_reduced_euler["ang_y"] = df["ang_y"] + 1.57
df_reduced_euler["ang_z"] = df["ang_z"]
df_reduced_euler["pos_x"] = df["pos_x"] +0.7
df_reduced_euler["pos_y"] = df["pos_y"] 
df_reduced_euler["pos_z"] = df["pos_z"] + 0.7

df_reduced_euler.to_csv("trajectory_euler2.csv", index= None ,header=None)