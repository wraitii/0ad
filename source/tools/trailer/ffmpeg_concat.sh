FRAMERATE=${FRAMERATE:="60"}
INPUT=${INPUT:="screens/videoscreen%04d.jpeg"}
OUTPUT=${OUTPUT:="trailer_crf15.mp4"}

# x264 constant rate factor. 0 is lossless, 23 default, 51 worst.
CONSTANT_RATE_FACTOR=${CRF:="15"}

# Scale to 1080p, size x (as a factor of two)
FILTER=${FILTER:="scale=-2:1080"}

ffmpeg \
	-start_number 0 \
	-framerate "$FRAMERATE" \
	-i "$INPUT" \
	-codec:v libx264 \
	-crf "$CONSTANT_RATE_FACTOR" \
	-filter:v "$FILTER" \
	"$OUTPUT"
