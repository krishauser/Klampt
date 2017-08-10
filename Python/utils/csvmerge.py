"""Merges two CSV files based on a matching key."""

import sys
import os

interpolate = False

if len(sys.argv) not in [4,5]:
    print "Usage: csvmerge file_a file_b [key] file_out"
    exit(0)

mergekey = "time"
filea = sys.argv[1]
fileb = sys.argv[2]
if len(sys.argv)==5:
    mergekey = sys.argv[3]
    outfn = sys.argv[4]
else:
    outfn = sys.argv[3]
headera = os.path.splitext(filea)[0]
headerb = os.path.splitext(fileb)[0]

try:
    import pandas as pd
    a = pd.read_csv(filea)
    b = pd.read_csv(fileb)
    a.columns = [(c if c==mergekey else headera+'.'+c) for c in a.columns]
    b.columns = [(c if c==mergekey else headerb+'.'+c) for c in b.columns]
    merged = a.merge(b, on=mergekey, how='outer')
    merged = merged.sort(columns=mergekey)
    if interpolate:
        timekeys = [pd.datetime.fromtimestamp(v) for v in merged[mergekey]]
        merged["__timestamps__"] = pd.Series(timekeys,index=merged.index)
        merged = merged.set_index("__timestamps__")
        for c in merged.columns:
            if c != mergekey:
                series = merged[c]
                newseries = series.interpolate(method='time')
                merged[c] = newseries
    merged.to_csv(outfn, index=False)
except ImportError:
    print "Need Pandas library..."
    raise

