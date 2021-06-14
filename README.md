# 0 A.D. A25 Trailer hacks

This contains some modifications to create the 0 A.D. A25 trailers.

All maps will automatically load the cinematics helper script, which will start a random cinematic.

Some have a dedicated replay:
- trailer_croco - the good stuff start around 40 seconds in, so don't record before that
- trailer_miletus
- trailer_sheep

You can easily make your own cinematics by using the following two console commands:
 - Get camera coordinates with `warn(JSON.stringify([Engine.GetCameraPosition(), Engine.GetCameraRotation()]))`
 - To get terrain coordinates: `warn(JSON.stringify(Engine.GetTerrainAtScreenPoint(mouseX, mouseY)));`.

You can just dump those in CinemaHelper.js and take inspiration from what's there.

To actually convert things into a video, you'll need to select an appropriate screenshot folder & convert it using
```sh source/tools/trailer/ffmpeg_concat.sh```
