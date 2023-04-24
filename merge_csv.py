import pandas as pd

data_one = pd.read_csv("./dataset/turn_with_1.0.csv")
data_two = pd.read_csv("./dataset/turn_with_2.0.csv")
data_three = pd.read_csv("./dataset/turn_with_3.0.csv")
data_four = pd.read_csv("./dataset/turn_with_4.0.csv")
data_five = pd.read_csv("./dataset/turn_with_5.0.csv")

# NOTE: saving this one for drifting data.
# data_six = pd.read_csv("./dataset/turn_with_6.0.csv")

# rewrite concat 
data = pd.concat([data_one, data_two, data_three, data_four, data_five], axis = 0)
data = data.reset_index(drop = True)
data = data.drop(columns=["Unnamed: 0"])

# extensive shuffling since there are multiple merges. 
# data = data.sample(frac=1).reset_index(drop=True)

# save merged csv
data.to_csv("./dataset/ikddata2.csv")