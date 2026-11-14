import sys
import zipfile
from collections import Counter
import json

path = r"C:\Users\34359\Desktop\RM资料\制导镖\飞镖飞塔.epro2"
with zipfile.ZipFile(path) as archive:
    lines = archive.read("sg2002.epru").decode("utf-8").splitlines()

print("records:", len(lines))
print("types:", Counter(json.loads(line.split("||", 1)[0]).get("type") for line in lines if "||" in line).most_common(20))
for number, line in enumerate(lines, 1):
    if line.startswith('{"type":"DOCHEAD"}') and number > 7000 and number < 30000:
        print("dochead", number, repr(line[:250]))
for token in sys.argv[1:]:
    matches = [(number, line) for number, line in enumerate(lines, 1)
               if number < 30000 and token.lower() in line.lower()]
    print("\nTOKEN", repr(token), "MATCHES", len(matches))
    for number, line in matches[:25]:
        at = line.lower().find(token.lower())
        print(number, repr(line[max(0, at - 180):at + 250]))
