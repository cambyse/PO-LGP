#include <SFML/Audio.hpp>
#include <cmath>
#include <iostream>
#include <Media/audio.h>

struct AudioStream:sf::InputStream {
  SineSound S;
  AudioStream(){}
  ~AudioStream(){}
  sf::Int64 read(void* data, sf::Int64 size){
    uint I = size/sizeof(int16_t);
    int16_t *p = (int16_t*)data;
    for(uint i=0;i<I;i++) p[i] = (int16_t) S.get();
    return size;
  }
  sf::Int64 seek(sf::Int64 position){ NIY; }
  sf::Int64 tell(){ NIY; }
  sf::Int64 getSize(){ return 0; }
};

int main() {
#if 1
  sf::Music music;
  AudioStream str;
  if (!music.openFromStream(str))
    return -1; // error
  music.play();
  str.S.addNote(440., .1, .0);
  mlr::wait(3.);
#else
    const unsigned SAMPLES = 44100;
    const unsigned SAMPLE_RATE = 44100;
    const unsigned AMPLITUDE = 30000;

    sf::Int16 raw[SAMPLES];

    const double TWO_PI = 6.28318;
    const double increment = 440./44100;
    double x = 0;
    for (unsigned i = 0; i < SAMPLES; i++) {
        raw[i] = AMPLITUDE * sin(x*TWO_PI);
        x += increment;
    }

    sf::SoundBuffer Buffer;
    if (!Buffer.loadFromSamples(raw, SAMPLES, 1, SAMPLE_RATE)) {
        std::cerr << "Loading failed!" << std::endl;
        return 1;
    }

    sf::Sound Sound;
    Sound.setBuffer(Buffer);
    Sound.setLoop(true);
    Sound.play();
    while (1) {
        sf::sleep(sf::milliseconds(100));
    }
#endif
    return 0;
}
