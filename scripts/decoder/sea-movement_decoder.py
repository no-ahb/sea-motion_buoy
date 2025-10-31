import struct

filename = "bno_009.bin"

# Header: magic(4) ver(1) rate(2) reserved(1) = 8 bytes
header_size = 8
# Record format: t_ms(4) lx(4) ly(4) lz(4) qi(4) qj(4) qk(4) qr(4) = 32 bytes
record_size = 32  # 4 + 7*4
record_format = "<Ifffffff"  # t_ms, lx, ly, lz, qi, qj, qk, qr

records = []

with open(filename, "rb") as f:
    data = f.read()
    
    # Skip the 8-byte header
    if len(data) < header_size:
        print(f"Error: File too short for header (got {len(data)} bytes)")
    else:
        # Read and verify header
        magic, ver, rate, reserved = struct.unpack("<IBHB", data[:header_size])
        print(f"File header: magic=0x{magic:08X} ('BNO1'), version={ver}, rate={rate} Hz")
        
        # Process records after header
        data_offset = header_size
        for i in range(data_offset, len(data), record_size):
            chunk = data[i:i+record_size]
            if len(chunk) == record_size:
                t_ms, lx, ly, lz, qi, qj, qk, qr = struct.unpack(record_format, chunk)
                records.append((t_ms, lx, ly, lz, qi, qj, qk, qr))

print(f"Read {len(records)} samples.")
print("First few samples:")
print("Format: (t_ms, lx, ly, lz, qi, qj, qk, qr)")
for r in records[:5]:
    print(r)
