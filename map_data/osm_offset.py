import xml.etree.ElementTree as ET

tree = ET.parse('lanelet2_map_lat47.osm')
root = tree.getroot()

offset = -47.0 # 내리고 싶은 높이

for node in root.findall('node'):
    for tag in node.findall('tag'):
        if tag.get('k') == 'ele':
            old_val = float(tag.get('v'))
            tag.set('v', str(old_val + offset))

tree.write('lanelet2_map.osm')
print("Done!")
