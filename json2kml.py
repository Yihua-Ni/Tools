import json
import simplekml

'''
 1 pip install simplekml
 2 修改jsonpath路径
'''

jsonpath = './shenzhenG107_RoadSign_WGS84.json'       


with open(jsonpath, 'r') as f:
    jsondata = json.loads(f.read())
    coord = jsondata['Rectangle Signs:']

kml = simplekml.Kml()
for key in coord:
    coord11, coord22 = [], []
    coord_pair = []
    coord1 = key['Coord'][0]
    coord2 = key['Coord'][1]


    coord11.append(coord1[1])
    coord11.append(coord1[0])
    coord11.append(coord1[2])


    coord22.append(coord2[1])
    coord22.append(coord2[0])
    coord22.append(coord2[2])


    coord_pair.append(coord11)
    coord_pair.append(coord22)
 

    lin = kml.newlinestring(name='pathway', description='A path way', coords=coord_pair)
    lin.style.linestyle.color = simplekml.Color.red                                            #轨迹颜色
    lin.style.linestyle.width = 6                                                              #轨迹粗细

kml.save('shenzhenG107_RoadSign_WGS84.kml')






