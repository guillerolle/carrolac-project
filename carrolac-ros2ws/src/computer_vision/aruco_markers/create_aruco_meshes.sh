#!/usr/bin/env bash

# ESTE SCRIPT CREA LOS .dae A PARTIR DEL AU0000.dae
# - COPIA LAS TEXTURAS DEL FRENTE Y REVERSO DEL ID NUEVO DESDE LA CARPETA 'markers'
# - DUPLICA AU0000.dae CON EL NUEVO ID
# - REEMPLAZA TODAS LAS INSTANCIAS DE AU0000 y AU0001 DENTRO DEL DAE POR LOS IDs DEL FRENTE Y REVERSO NUEVOS

for i in {2..5}
do
  ARUCO_STRING="$(printf "AU%0.4i\n" "$(("$i"*2))")"
  ARUCO_STRING_BACKFACE="$(printf "AU%0.4i\n" "$(("$i"*2 + 1))")"
  echo "Creating ArUco mesh $ARUCO_STRING..."
  # cp "meshes/AU0000.dae" "meshes/$ARUCO_STRING.dae"
  cp "markers/$ARUCO_STRING.png" "meshes/$ARUCO_STRING.png"
  cp "markers/$ARUCO_STRING_BACKFACE.png" "meshes/$ARUCO_STRING_BACKFACE.png"
  sed -e "s/AU0000/$ARUCO_STRING/g" -e "s/AU0001/$ARUCO_STRING_BACKFACE/g" "meshes/AU0000.dae" \
    > "meshes/$ARUCO_STRING.dae"
done

