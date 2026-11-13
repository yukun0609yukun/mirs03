# source して使う。ROS 2(Fast DDS)の通信を有線NIC(enP8p1s0)だけに制限する。
#   無効化(ケーブル抜け時の緊急用): NOKOSAHEN_DDS_WIRED=0 を付けて起動 / bashrcなら一行 export NOKOSAHEN_DDS_WIRED=0
#   (有線を抜くと、無効化しない限りJetson同士のROS通信が途切れる)
_nk_xml="${NOKOSAHEN_DDS_XML:-$HOME/mirs2603_nokosahen/config/fastdds_wired.xml}"
if [ "${NOKOSAHEN_DDS_WIRED:-1}" = "0" ] || [ ! -f "$_nk_xml" ]; then
    unset FASTRTPS_DEFAULT_PROFILES_FILE RMW_FASTRTPS_USE_QOS_FROM_XML
else
    export FASTRTPS_DEFAULT_PROFILES_FILE="$_nk_xml"
    export RMW_FASTRTPS_USE_QOS_FROM_XML=0
fi
unset _nk_xml
