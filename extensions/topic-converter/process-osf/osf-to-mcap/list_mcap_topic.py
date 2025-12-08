from mcap.reader import make_reader

path = "Traffic_Intersection_raw.mcap"
with open(path, "rb") as f:
    r = make_reader(f)
    summary = r.get_summary()

topics = sorted({ch.topic for ch in summary.channels.values()})
print("Topics:")
for t in topics:
    print(" -", t)
