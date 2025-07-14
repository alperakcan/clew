# clew
Clew is a semi-optimal route planning tool for adventure motor touring — (aka: Ariadne’s guiding thread).

# usage

```
clew usage:
  --input              / -i : input path
  --output             / -o: output path
  --clip-path          / -c: clip path, ex: lon1,lat1 lon2,lat2 ... (default: "")
  --clip-bound             : clip bound, ex: minlon,minlat,maxlon,maxlat (default: "")
  --clip-offset            : clip offset in meters (default: 0)
  --filter             / -f: filter expression (default: "")
  --points             / -p: points to visit, ex: lon1,lat1 lon2,lat2 ... (default: "")
  --keep-nodes         / -n: filter nodes (default: 0)
  --keep-ways          / -w: filter ways (default: 0)
  --keep-relations     / -r: filter relations (default: 0)
  --keep-tags          / -k: keep tag (default: "")
  --keep-tags-node         : keep node tag (default: "")
  --keep-tags-way          : keep way tag (default: "")
  --keep-tags-relation     : keep relation tag (default: "")
```

```
clew usage:

  --input              / -i : input path
  --output             / -o: output path
  --expression         / -e: filter expression
  --keep-nodes         / -n: filter nodes (default: 0)
  --keep-ways          / -w: filter ways (default: 0)
  --keep-relations     / -r: filter relations (default: 0)
  --keep-tag               : keep tag (default: 0)
  --keep-tag-node          : keep node tag (default: 0)
  --keep-tag-way           : keep way tag (default: 0)
  --keep-tag-relation      : keep relation tag (default: 0)

example:

  clew --input input.osm.pbf --output output.pgx --expression expression
  clew --input input.osm.pbf --output output.pgx --expression highway_*
  clew --input input.osm.pbf --output output.pgx
    --keep_ways 1
    --expression "highway_motorway or highway_motorway_link or highway_trunk or highway_trunk_link or highway_primary"
    --keep-tag highway_motorway --keep-tag highway_motorway_link
    --keep-tag highway_trunk --keep-tag highway_trunk_link
    --keep-tag highway_primary

roads:
  highway_motorway    : A restricted access major divided highway, normally with 2 or more running lanes plus emergency hard shoulder. Equivalent to the Freeway, Autobahn, etc..
  highway_trunk       : The most important roads in a country's system that aren't motorways. (Need not necessarily be a divided highway.
  highway_primary     : The next most important roads in a country's system. (Often link larger towns.)
  highway_secondary   : The next most important roads in a country's system. (Often link towns.)
  highway_tertiary    : The next most important roads in a country's system. (Often link smaller towns and villages)
  highway_unclassified: The least important through roads in a country's system – i.e. minor roads of a lower classification than tertiary, but which serve a purpose other than access to properties. (Often link villages and hamlets.)
  highway_residential : Roads which serve as an access to housing, without function of connecting settlements. Often lined with housing.

link roads:
  highway_motorway_link : The link roads (sliproads/ramps) leading to/from a motorway from/to a motorway or lower class highway. Normally with the same motorway restrictions.
  highway_trunk_link    : The link roads (sliproads/ramps) leading to/from a trunk road from/to a trunk road or lower class highway.
  highway_primary_link  : The link roads (sliproads/ramps) leading to/from a primary road from/to a primary road or lower class highway.
  highway_secondary_link: The link roads (sliproads/ramps) leading to/from a secondary road from/to a secondary road or lower class highway.
  highway_tertiary_link : The link roads (sliproads/ramps) leading to/from a tertiary road from/to a tertiary road or lower class highway.

special road types:
  highway_living_street: For living streets, which are residential streets where pedestrians have legal priority over cars, speeds are kept very low.
  highway_service      : For access roads to, or within an industrial estate, camp site, business park, car park, alleys, etc.
  highway_pedestrian   : For roads used mainly/exclusively for pedestrians in shopping and some residential areas which may allow access by motorised vehicles only for very limited periods of the day.
  highway_track        : Roads for mostly agricultural or forestry uses.
  highway_bus_guideway : A busway where the vehicle guided by the way (though not a railway) and is not suitable for other traffic.
  highway_escape       : For runaway truck ramps, runaway truck lanes, emergency escape ramps, or truck arrester beds.
  highway_raceway      : A course or track for (motor) racing
  highway_road         : A road/way/street/motorway/etc. of unknown type. It can stand for anything ranging from a footpath to a motorway.
  highway_bus_way      : A dedicated roadway for bus rapid transit systems
paths:
  highway_footway      : For designated footpaths; i.e., mainly/exclusively for pedestrians. This includes walking tracks and gravel paths.
  highway_bridleway    : For horse riders. Pedestrians are usually also permitted, cyclists may be permitted depending on local rules/laws. Motor vehicles are forbidden.
  highway_steps        : For flights of steps (stairs) on footways.
  highway_corridor     : For a hallway inside of a building.
  highway_path         : A non-specific path.
  highway_via_ferrata  : A via ferrata is a route equipped with fixed cables, stemples, ladders, and bridges in order to increase ease and security for climbers.
```
