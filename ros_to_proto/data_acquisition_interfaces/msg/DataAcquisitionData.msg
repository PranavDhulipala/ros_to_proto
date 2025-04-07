# Complete ROS primitive types and variations

# ---- Primitives (not array) ----
# ---- FIELDS (lowercase by convention) ----
# Boolean type
bool bool_value

# Integer types (signed and unsigned)
int8 int8_value
uint8 uint8_value
byte byte_value          # alias for uint8
char char_value          # typically for single characters

int16 int16_value
uint16 uint16_value

int32 int32_value
uint32 uint32_value

int64 int64_value
uint64 uint64_value

# Floating point types
float32 float32_value
float64 float64_value    # double precision

# String types
string string_value
string<=10 bounded_string_value  # String with max length of 10
wstring wstring_value   # Wide string for unicode support
wstring<=5 bounded_wstring_value  # Wide string with max length of 5

bool[3] bool_fixed_array
int8[4] int8_fixed_array
uint8[5] uint8_fixed_array
byte[2] byte_fixed_array
char[10] char_fixed_array
int16[3] int16_fixed_array
uint16[3] uint16_fixed_array
int32[3] int32_fixed_array
uint32[3] uint32_fixed_array
int64[3] int64_fixed_array
uint64[3] uint64_fixed_array
float32[3] float32_fixed_array
float64[3] float64_fixed_array
string[3] string_fixed_array
wstring[3] wstring_fixed_array

# Unbounded dynamic arrays (variable length)
bool[] bool_unbounded_array
int8[] int8_unbounded_array
uint8[] uint8_unbounded_array
byte[] byte_unbounded_array
char[] char_unbounded_array
int16[] int16_unbounded_array
uint16[] uint16_unbounded_array
int32[] int32_unbounded_array
uint32[] uint32_unbounded_array
int64[] int64_unbounded_array
uint64[] uint64_unbounded_array
float32[] float32_unbounded_array
float64[] float64_unbounded_array
string[] string_unbounded_array
wstring[] wstring_unbounded_array

# Bounded dynamic arrays (variable length with a maximum)
bool[<=5] bool_bounded_array
int8[<=5] int8_bounded_array
uint8[<=5] uint8_bounded_array
byte[<=5] byte_bounded_array
char[<=5] char_bounded_array
int16[<=5] int16_bounded_array
uint16[<=5] uint16_bounded_array
int32[<=5] int32_bounded_array
uint32[<=5] uint32_bounded_array
int64[<=5] int64_bounded_array
uint64[<=5] uint64_bounded_array
float32[<=5] float32_bounded_array
float64[<=5] float64_bounded_array
string[<=5] string_bounded_array
wstring[<=5] wstring_bounded_array

key_value_interfaces/KeyValuePair[] sample_map