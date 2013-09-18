#!/bin/sh

DEMOS='5'
SEEDS="1 5"
GPSIZES='.1 .4 .7 1'
OBJSIZES="1"
SIGMAS=".1 1 10"
RADIUSES=".05 .1 .5"
observs=50

echo  demo:$DEMOS  seed:$SEEDS gpsize:$GPSIZES sigma:$SIGMAS radius:$RADIUSES objsize:$OBJSIZES

for demo in $DEMOS; do
  for seed in $SEEDS; do
    for gpsize in $GPSIZES; do
      for sigma in $SIGMAS; do
        for radius in $RADIUSES; do
          for objsize in $OBJSIZES; do
            fn="$demo-$seed-$gpsize-$objsize-$sigma-$radius"
            # don't forget  -b y !
            qsub -cwd  -b y -o "$fn.log"  -e "$fn.e" \
              "$HOME"/mlr/stanio/share-branch/test/demo-GP_ImplicitSurface/x.exe \
                 -demo $demo \
                 -gp_size $gpsize \
                 -rnd_srfc_seed $seed \
                 -observations $observs \
                 -gl_auto 1 \
                 -with_GL 0 \
                 -center '"[0,0,0]"' \
                 -objsize $objsize \
                 -sigma $sigma \
                 -radius $radius \
                 -height .25 \
                 -finalbelieffile "$fn-finalbelief.tri" \
                 -showeveryith 10 \
                 -showuntil $observs
          done;
        done;
      done;
    done;
    sleep 200;
  done
done
