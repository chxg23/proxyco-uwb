#!/bin/bash

function process_pkgyml ()
{
    name=$1

    perl -i -pe 's/\@ultrawideband-apps\///g;' $name
    perl -i -pe 's/\@ultrawideband-core\///g;' $name
    perl -i -pe 's/\@ultrawideband-dw3000-c0\///g;' $name
}


function usage {
    echo "Usage $0 <src path> <dst path>"
    exit 2
}

function dir_sync {
    SRCP=$1
    SRCD=$2
    DSTP=$3
    echo "    dir_sync $1 $2 $3"
    for S in $(find $SRCD -type f |grep -e "\.yml$" -e "\.s$" -e "\.c$" -e "\.h$" -e "\.sh$" -e "\.cmd$" -e "\.ld$"|grep -v -e "\.o\.cmd$");do
        F=$(echo $S | sed "s|$SRCP/||")
        echo "    mkdir -p $(dirname $DSTP/$F)"
        echo "    cp $SRCP/$F $DSTP/$F"
        mkdir -p $(dirname $DSTP/$F)
        cp $SRCP/$F $DSTP/$F
    done
}

SRC=$1
if [ -z "$1" ]; then
    usage
fi

DST=$2
if [ -z "$2" ]; then
    usage
fi

#Revdep newt target revdep pca10095_twr_aloha_tag | grep -v -e apache-mynewt- -e mcuboot -e targets -e "Reverse dep"|cut -f6 -d" "
REVDEP="@ultrawideband-core/hw/drivers/uwb @ultrawideband-core/lib/dsp @ultrawideband-core/lib/euclid @ultrawideband-core/lib/json @ultrawideband-core/lib/rng_math @ultrawideband-core/lib/twr_ds @ultrawideband-core/lib/twr_ds_ext @ultrawideband-core/lib/twr_ss @ultrawideband-core/lib/twr_ss_ack @ultrawideband-core/lib/twr_ss_ext @ultrawideband-core/lib/uwb_rng @ultrawideband-core/porting/dpl/mynewt @ultrawideband-core/porting/dpl_lib @ultrawideband-core/sys/uwbcfg @ultrawideband-dw3000-c0/hw/bsp/pca10095_dw3000 @ultrawideband-dw3000-c0/hw/drivers/uwb/uwb_dw3000-c0 apps/twr_aloha"
#REVDEP="@ultrawideband-core/hw/drivers/uwb @ultrawideband-dw3000-c0/hw/drivers/uwb/uwb_dw3000-c0 apps/twr_aloha"

for R in $REVDEP;do
    echo "$R"
    if [[ "${R:0:1}" == '@' ]];then
        F=$(echo $R | sed "s|^[^/]*/||")
        SF=$(echo $R | sed "s|@||")
        echo "  $SRC/repos/$SF $DST/$F"
        dir_sync $SRC/repos/$SF $SRC/repos/$SF $DST/$F
    else
        F=$(echo $R)
        SF=$(echo $R | sed "s|@||")
        echo "  $SRC/repos/$SF $DST/$F"
        dir_sync $SRC/$SF $SRC/$SF $DST/$F
    fi
done

cp -r $SRC/targets/pca10095_twr_aloha_tag $DST/targets/
cp -r $SRC/targets/pca10095_twr_aloha_node $DST/targets/

for F in $(find $DST -type f -name "pkg.yml");do
    process_pkgyml $F
done

for F in $(find $DST/targets -type f -name "target.yml");do
    process_pkgyml $F
done

find ${DST}/ -name "*~"|xargs rm
