import sys
import xml.etree.ElementTree as elemTree

tree = elemTree.parse('A2_LINK.gml')
root = tree.getroot()

paths_posList = []

for child in root.iter("{http://www.opengis.net/gml}posList"):
  paths_posList.append(child.text)

paths_ID = []

for child in root.iter("{http://www.ngii.go.kr}ID"):
  paths_ID.append(child.text)

paths = {}

for ID, posList in zip(paths_ID, paths_posList):
  paths[ID] = posList


# 특정 링크만 파싱
for path_id in sys.argv[1:]:
  f = open(path_id + ".txt", 'w')
  path = paths[path_id].split()

  i = 0
  while i <= len(path) - 2:
    f.write(path[i] + ' ' + str(path[i+1]) + '\n')
    i += 2

  f.close()

# 모든 링크 파싱
# for path_id in paths_ID:
#   f = open(path_id + ".txt", 'w')
#   path = paths[path_id].split()
#
#   i = 0
#   while i <= len(path) - 2:
#     f.write(path[i] + ' ' + str(path[i+1]) + '\n')
#     i += 2
#
#   f.close()

