#!/bin/bash

#================================================================#
# Copyright (c) 2010-2011 Zipline Games, Inc.
# All Rights Reserved.
# http://getmoai.com
#================================================================#

set -e

cd `dirname $0`/

if [ x"$ANDROID_NDK" == x ]; then
    echo "ANDROID_NDK not defined. Please set to the location of your Android NDK install (path)"
    exit 1
fi
# check for command line switches
usage="usage: $0  \
    [--use-untz true | false] [--use-luajit true | false] [--disable-adcolony] [--disable-billing] \
    [--disable-chartboost] [--disable-crittercism] [--disable-facebook] [--disable-push] [--disable-tapjoy] \
    [--disable-twitter] [--disable-playservices] [--windows] [--release] [--incremental] \
    [--architecture <arch as in android.toolchain.cmake>]"

use_untz="true"
use_luajit="false"

adcolony_flags=
billing_flags=
chartboost_flags=
crittercism_flags=
facebook_flags=
push_flags=
tapjoy_flags=
twitter_flags=
buildtype_flags="Debug"
windows_flags=
playservices_flags=
incremental="false"
architecture="armeabi-v7a"

while [ $# -gt 0 ]; do
    case "$1" in
        --use-untz)  use_untz="$2"; shift;;
        --use-luajit)  use_luajit="$2"; shift;;
        --disable-adcolony)  adcolony_flags="-DDISABLE_ADCOLONY";;
        --disable-billing)  billing_flags="-DDISABLE_BILLING";;
        --disable-chartboost)  chartboost_flags="-DDISABLE_CHARTBOOST";;
        --disable-crittercism)  crittercism_flags="-DDISABLE_CRITTERCISM";;
        --disable-facebook)  facebook_flags="-DDISABLE_FACEBOOK";;
        --disable-push)  push_flags="-DDISABLE_NOTIFICATIONS";;
        --disable-tapjoy)  tapjoy_flags="-DDISABLE_TAPJOY";;
        --disable-twitter)  twitter_flags="-DDISABLE_TWITTER";;
        --disable-playservices)  playservices_flags="-DDISABLE_PLAYSERVICES";;
        --release) buildtype_flags="Release";;
        --windows) windows_flags=-G"MinGW Makefiles";; 
        --incremental) incremental="true";;
        --architecture) architecture="$2"; shift;;
        -*)
            echo >&2 \
                $usage
            exit 1;;
        *)  break;; # terminate while loop
    esac
    shift
done
make_flags=
if [ x"$windows_flags" != x ]; then
  make_flags=-DCMAKE_MAKE_PROGRAM="${ANDROID_NDK}/prebuilt/windows-x86_64/bin/make.exe"
fi

if [ x"$use_untz" != xtrue ] && [ x"$use_untz" != xfalse ]; then
    echo $usage
    exit 1      
fi

if [ x"$use_luajit" != xtrue ] && [ x"$use_luajit" != xfalse ]; then
    echo $usage
    exit 1      
fi

# echo message about what we are doing
echo "Building libmoai.so via CMAKE"

disabled_ext=
    


if [ x"$use_untz" != xtrue ]; then
    echo "UNTZ will be disabled"
    untz_param='-DMOAI_UNTZ=0'
else
    untz_param='-DMOAI_UNTZ=1'
fi 

if [ x"$use_luajit" != xtrue ]; then
    echo "LUAJIT will be disabled"
    luajit_param='-DMOAI_LUAJIT=0'
else
    luajit_param='-DMOAI_LUAJIT=1'
fi 

if [ x"$adcolony_flags" != x ]; then
    echo "AdColony will be disabled"
    disabled_ext="$disabled_extADCOLONY;"
fi 

if [ x"$billing_flags" != x ]; then
    echo "Billing will be disabled"
    disabled_ext="$disabled_extBILLING;"
fi 

if [ x"$chartboost_flags" != x ]; then
    echo "ChartBoost will be disabled"
    disabled_ext="$disabled_extCHARTBOOST;"
fi 

if [ x"$crittercism_flags" != x ]; then
    echo "Crittercism will be disabled"
    disabled_ext="$disabled_extCRITTERCISM;"
fi 

if [ x"$facebook_flags" != x ]; then
    echo "Facebook will be disabled"
    disabled_ext="$disabled_extFACEBOOK;"
fi 

if [ x"$push_flags" != x ]; then
    echo "Push Notifications will be disabled"
    disabled_ext="$disabled_extNOTIFICATIONS;"
fi 

if [ x"$tapjoy_flags" != x ]; then
    echo "Tapjoy will be disabled"
    disabled_ext="$disabled_extTAPJOY;"
fi 

if [ x"$twitter_flags" != x ]; then
    echo "Twitter will be disabled"
    disabled_ext="$disabled_extTWITTER;"
fi 

cd ../../cmake

build_dir_name="build-android-${architecture}"

if [ x"$incremental" == xfalse ]; then
    rm -rf $build_dir_name
fi

mkdir -p $build_dir_name
cd $build_dir_name
 
#create our makefiles
cmake -DDISABLED_EXT="$disabled_ext" -DMOAI_BOX2D=0 \
-DMOAI_CHIPMUNK=0 -DMOAI_CURL=1 -DMOAI_CRYPTO=1 -DMOAI_EXPAT=1 -DMOAI_FREETYPE=1 \
-DMOAI_HTTP_CLIENT=1 -DMOAI_JSON=1 -DMOAI_JPG=1 -DMOAI_LUAEXT=1 \
-DMOAI_MONGOOSE=1 -DMOAI_OGG=1 -DMOAI_OPENSSL=1 -DMOAI_SQLITE3=1 \
-DMOAI_TINYXML=1 -DMOAI_PNG=1 -DMOAI_SFMT=1 -DMOAI_VORBIS=1 $untz_param $luajit_param \
-DBUILD_ANDROID=true \
-DCMAKE_TOOLCHAIN_FILE="../host-android/android.toolchain.cmake" \
-DLIBRARY_OUTPUT_PATH_ROOT="../../ant/libmoai" \
-DANDROID_NDK=${ANDROID_NDK} \
-DANDROID_ABI="${architecture}" \
-DCMAKE_BUILD_TYPE=$buildtype_flags \
"${windows_flags}" "${make_flags}" \
-DPLUGIN_SKYTURNS-GEOMETRY-GENERATOR=1 \
-DPLUGIN_MOAI-FMOD-STUDIO=1 \
-DPLUGIN_MOAI-HOCKEYAPP-ANDROID=1 \
-DPLUGIN_MOAI-GETTOUCHES-ANDROID=1 \
-DPLUGIN_SKYTURNS-INFO=1 \
-DPLUGIN_DIR=E:/dev/projekt/skyturns/moai-plugins \
../
#build them    
if [ x"$windows_flags" != x ]; then
  cmake --build . --target moai
else
  cmake --build . --target moai -- -j4
fi  