# clew
Clew is a semi-optimal route planning tool for adventure motor touring — (aka: Ariadne’s guiding thread).

# usage

    CLEW_MIMALLOC_ENABLE=n \
    make && \
    ./dist/bin/clew \
      -i ../patika-maps/europe-latest-clipped-100000-highways.osm.pbf \
      -o o \
      -f "highway_*" \
      --keep-tags-node "" \
      --keep-tags-way "highway_* or tunnel_* or bridge_* or toll_* or oneway_*" \
      --keep-tags-relation "" \
      --keep-nodes 0 \
      --keep-ways 1 \
      --keep-relations 0 \
      -p "12.1358000,46.5405000 12.0855000,46.6945000 12.8428000,47.0401000 12.8333000,47.0833000 12.1697000,47.2075000 10.6949850,46.3496280 10.4846264,46.3434911 10.3699000,46.4670000 10.4522814,46.5284730 10.5445347,46.5950134 10.5138764,46.8286048 10.1357000,46.5386000 8.5698617,46.5464946 8.4104009,46.5718475 7.9071000,46.5930000 7.9106000,46.5621000 8.4464581,46.7295143 8.5956753,46.6376523 8.3706713,46.4772018 7.1973672,45.9025618 7.0305294,45.4177072 6.4076548,45.0641193 9.3301869,46.5060171 6.6364000,45.6925000 8.8758000,46.8778000 10.2983079,46.2479261" \
      -c 100000

    osmium extract \
      --progress \
      -b 5.1359257,44.1658040,14.1608461,48.1058152 \
      -s smart \
      -o europe-latest-clipped-100000m.osm.pbf \
      europe-latest.osm.pbf \

    osmium tags-filter \
      --progress \
      -o europe-latest-clipped-100000-highways.osm.pbf \
      europe-latest-clipped-100000.osm.pbf \
      nwr/highway r/restriction

    ogr2ogr -f KML output-waypoints.kml output.gpx waypoints
    ogr2ogr -f KML output-tracks.kml output.gpx tracks
    ogr2ogr -f KML output.kml output-waypoints.kml
    ogr2ogr -f KML -append output.kml output-tracks.kml
    ogr2ogr -f KMLSUPEROVERLAY output.kmz output.kml
    zip -j output.kmz output.kml
