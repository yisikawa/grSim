import os
import sys
from pathlib import Path

# protoc 3.12.4 (this repo's system protoc, via apt's protobuf-compiler)
# generates descriptor-based Python code that the protobuf pip package's
# default (upb) backend refuses to load ("Descriptors cannot be created
# directly."). Force the pure-Python implementation instead. This MUST be
# set before any generated _pb2 module is imported anywhere in this package.
os.environ.setdefault("PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION", "python")

_PB_DIR = Path(__file__).resolve().parent / "pb"
if str(_PB_DIR) not in sys.path:
    sys.path.insert(0, str(_PB_DIR))
