import pandas as pd

df = pd.read_parquet('./lerobot_dataset/data/chunk_000/episode_000001.parquet')
# print(df)
print("📦 Keys (columns):")
print(df.columns)

print("\n🔍 Values for each key:")
for col in df.columns:
    print(f"\n📌 {col}:")
    print(df[col].values)
