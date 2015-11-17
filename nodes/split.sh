#!/bin/bash 
# coding: utf8
DEBUG=1
# First write the buffer stream to .buff files (annotated using GStreamer's GDP format)
echo "begin first launch!!"
gst-launch-0.10 -e gconfaudiosrc ! audioconvert ! audioresample ! gdppay ! multifilesink next-file=4 max-file-size=100000 location=foo%05d.buff &
#gst-launch-0.10 -e gconfaudiosrc !audioconvert ! audioresample !filesink location=music.wav ! wavparse ! gdppay ! multifilesink next-file=4 max-file-size=100000 location=foo%05d.buff
#gst-launch-0.10 -e filesrc location=music.wav ! wavparse ! gdppay ! multifilesink next-file=4 max-file-size=100000 location=foo%05d.buff
# use the following instead for any other source (e.g. internet radio streams)
#gst-launch -e uridecodebin uri=http://url.to/stream ! gdppay ! multifilesink next-file=4 max-file-size=1000000 location=foo%05d.buff

# After we're done, convert each of the resulting files to proper .wav files with headers
for file in *.buff; do
    tgtFile="$(echo "$file"|sed 's/.buff$/.wav/')"
    echo $tgtFile
    gst-launch-0.10 -e filesrc "location=$file" ! gdpdepay ! wavenc ! filesink "location=$tgtFile" &
done
    
    # Uncomment the following line to remove the .buff files here, but to avoid accidentally 
    # deleting stuff we haven't properly converted if something went wrong, I'm not gonna do that now.
    #rm *.buff''"""""''"")"'')
