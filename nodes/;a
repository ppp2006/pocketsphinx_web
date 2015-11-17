#!/usr/bin/python 
# First write the buffer stream to .buff files (annotated using GStreamer's GDP format)
gst-launch-0.10  -e filesrc location=/home/ppeix/music.wav ! wavparse ! gdppay ! multifilesink next-file=4 max-file-size=1000000 location=/home/ppeix/foo%05d.buff
# use the following instead for any other source (e.g. internet radio streams)
#gst-launch -e uridecodebin uri=http://url.to/stream ! gdppay ! multifilesink next-file=4 max-file-size=1000000 location=foo%05d.buff

# After we're done, convert each of the resulting files to proper .wav files with headers
for file in *.buff; do
    tgtFile="$(echo "$file"|sed 's/.buff$/.wav/')"
    
        gst-launch-0.10 filesrc "location=$file" ! gdpdepay ! wavenc ! filesink "location=$tgtFile"
    done
    
    # Uncomment the following line to remove the .buff files here, but to avoid accidentally 
    # deleting stuff we haven't properly converted if something went wrong, I'm not gonna do that now.
    #rm *.buff''"""""''"")"'')
