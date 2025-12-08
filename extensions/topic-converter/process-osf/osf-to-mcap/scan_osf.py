from ouster.sdk import osf

src = osf.OsfScanSource("Traffic_Intersection.osf")

item = next(iter(src))
scans = item if isinstance(item, (list, tuple)) else [item]

for i, scan in enumerate(scans):
    print(f"\n--- scan {i} ---")
    print("fields:", scan.fields)
    for f in scan.fields:
        arr = scan.field(f)
        print(f, arr.dtype, arr.shape)
