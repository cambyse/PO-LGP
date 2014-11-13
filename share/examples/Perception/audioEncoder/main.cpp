#include <Perception/audio.h>

void TEST(Audio){
    AudioPoller_PA poller;
    AudioWriter_libav writer("test.wav");

    byteA buf; buf.resize(4096);

    while(poller.read(buf)) {
        writer.writeSamples_R48000_2C_S16_NE(buf);
    }
}

int main(int argc,char **argv){
    testAudio();

    return 0;
}

