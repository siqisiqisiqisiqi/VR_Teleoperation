import pandas as pd

# Read the Parquet file
df = pd.read_parquet("./lerobot_dataset/data/chunk-000/episode_000000.parquet")

# Print the entire DataFrame
# print(df)

# # Optionally: print just the first few rows
# print(df.head())

# Print the first row
print(df.iloc[0].to_dict())