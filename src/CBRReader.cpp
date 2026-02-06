#include "CBRReader.h"
#include <stdexcept>


CBRReader::CBRReader () = default;

CBRReader::~CBRReader () = default;

void CBRReader::setupCBRReader(bool verbose, std::string dissemination_interface)
{
    m_cbr_data.verbose = verbose;
    m_dissemination_interface = dissemination_interface;
}

float CBRReader::get_current_cbr()
{
    return m_cbr_data.currentCBR;
}

float CBRReader::get_current_busy_time()
{
    return m_cbr_data.currentBusyTime;
}

float CBRReader::get_current_tx_time()
{
    return m_cbr_data.currentTxTime;
}

float CBRReader::get_current_rx_time()
{
    return m_cbr_data.currentRxTime;
}

void CBRReader::start_reading_cbr(nl_sock_info_t m_nl_sock_info)
{
    pthread_setname_np(pthread_self(), "CBR_reader_thr");

    if (m_nl_sock_info.sock_valid==true)
    {
        read_cbr_from_netlink(m_nl_sock_info);
    }
    else
    {
        throw std::runtime_error("Socket is invalid. Cannot read from netlink.");
    }
}

void CBRReader::read_cbr_from_netlink(nl_sock_info_t nl_sock_info)
{
    struct nl_msg *nl_msg = nlmsg_alloc();

	genlmsg_put(nl_msg,NL_AUTO_PORT,NL_AUTO_SEQ,nl_sock_info.nl80211_id,0,NLM_F_DUMP | NLM_F_REQUEST,NL80211_CMD_GET_SURVEY,0);
	nla_put_u32(nl_msg,NL80211_ATTR_IFINDEX,nl_sock_info.ifindex);

	nl_socket_modify_cb(nl_sock_info.nl_sock,NL_CB_VALID,NL_CB_CUSTOM,cbr_handler,this);

    nl_send_auto_complete(nl_sock_info.nl_sock,nl_msg);

    int err = nl_recvmsgs_default(nl_sock_info.nl_sock);

    free_nl_socket(nl_sock_info);
}

void CBRReader::wrapper_start_reading_cbr()
{
    nl_sock_info_t nl_sock_info = open_nl_socket(m_dissemination_interface);
    m_cbr_thread = std::thread(&CBRReader::start_reading_cbr, this, nl_sock_info);
    if(m_cbr_thread.joinable())
    {
        m_cbr_thread.join();
    }
    return;
}

int cbr_handler(struct nl_msg *msg, void *arg)
{
    auto* reader = reinterpret_cast<CBRReader*>(arg);
    struct nlattr *tb[NL80211_ATTR_MAX + 1];
    struct genlmsghdr *gnlh = (struct genlmsghdr *) nlmsg_data(nlmsg_hdr(msg));
    struct nlattr *sinfo[NL80211_SURVEY_INFO_MAX + 1];
    char dev[20];

    static struct nla_policy survey_policy[NL80211_SURVEY_INFO_MAX + 1] = {};
    survey_policy[NL80211_SURVEY_INFO_FREQUENCY].type = NLA_U32;
    survey_policy[NL80211_SURVEY_INFO_NOISE].type = NLA_U8;

    nla_parse(tb, NL80211_ATTR_MAX, genlmsg_attrdata(gnlh, 0), genlmsg_attrlen(gnlh, 0), NULL);

    if (!tb[NL80211_ATTR_SURVEY_INFO]) {
        fprintf(stderr, "Survey data missing!\n");
        return NL_SKIP;
    }

    if (nla_parse_nested(sinfo, NL80211_SURVEY_INFO_MAX, tb[NL80211_ATTR_SURVEY_INFO], survey_policy)) {
        fprintf(stderr, "Failed to parse nested survey attributes!\n");
        return NL_SKIP;
    }

    unsigned long long activeTime = 0, busyTime = 0, rxTime = 0, txTime = 0;

    if (!sinfo[NL80211_SURVEY_INFO_IN_USE])
    {
        return NL_SKIP;  // Not the active channel
    }

    if (sinfo[NL80211_SURVEY_INFO_FREQUENCY] && reader->m_cbr_data.verbose && reader->m_cbr_data.firstTime)
    {
        printf("Frequency:\t\t\t%u MHz%s\n", nla_get_u32(sinfo[NL80211_SURVEY_INFO_FREQUENCY]),
               sinfo[NL80211_SURVEY_INFO_IN_USE] ? " [in use]" : "");
    }

    if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME])
        activeTime = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME]);

    if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_BUSY])
        busyTime = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_BUSY]);

    if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_RX])
        rxTime = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_RX]);

    if (sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_TX])
        txTime = nla_get_u64(sinfo[NL80211_SURVEY_INFO_CHANNEL_TIME_TX]);

    bool firstTime = reader->m_cbr_data.firstTime;

    if (firstTime) {
        if (reader->m_cbr_data.verbose)
        {
            std::cout << "First Time for CBR Computing" << std::endl;
        }
        reader->m_cbr_data.firstTime = false;
        reader->m_cbr_data.startActiveTime = activeTime;
        reader->m_cbr_data.startBusyTime = busyTime;
        reader->m_cbr_data.startReceiveTime = rxTime;
        reader->m_cbr_data.startTransmitTime = txTime;
        reader->m_cbr_data.currentCBR = -1.0f;
        return NL_SKIP;
    }

    // Check for counter reset or wraparound
    if (activeTime < reader->m_cbr_data.startActiveTime || busyTime < reader->m_cbr_data.startBusyTime ||
        rxTime < reader->m_cbr_data.startReceiveTime || txTime < reader->m_cbr_data.startTransmitTime) {
        if (reader->m_cbr_data.verbose)
        {
            std::cerr << "Warning: time counters reset or wrapped. Skipping...\n";
        }
        reader->m_cbr_data.startActiveTime = activeTime;
        reader->m_cbr_data.startBusyTime = busyTime;
        reader->m_cbr_data.startReceiveTime = rxTime;
        reader->m_cbr_data.startTransmitTime = txTime;
        return NL_SKIP;
    }

    unsigned long long deltaActiveTime = activeTime - reader->m_cbr_data.startActiveTime;
    unsigned long long deltaBusyTime = busyTime - reader->m_cbr_data.startBusyTime;
    unsigned long long deltaReceiveTime = rxTime - reader->m_cbr_data.startReceiveTime;
    unsigned long long deltaTransmitTime = txTime - reader->m_cbr_data.startTransmitTime;

    if (reader->m_cbr_data.verbose)
    {
        std::cout << "Active-time: " << deltaActiveTime
                  << " Busy-time: " << deltaBusyTime
                  << " RX-time: " << deltaReceiveTime
                  << " TX-time: " << deltaTransmitTime << std::endl;
    }

    float cbr = -1.0f;

    if (deltaActiveTime > 0) {
        if (deltaBusyTime < 0) deltaBusyTime = 0;
        cbr = static_cast<float>(deltaBusyTime) / deltaActiveTime;
        if (cbr < 0.0f) cbr = 0.0f;
        if (cbr > 1.0f) cbr = 1.0f;
    }

    if (reader->m_cbr_data.verbose && cbr >= 0.0f)
        std::cout << "Current Channel Busy Ratio: " << cbr << std::endl;

    // Save new base values for next delta
    reader->m_cbr_data.currentCBR = cbr;
    reader->m_cbr_data.startActiveTime = activeTime;
    reader->m_cbr_data.startBusyTime = busyTime;
    reader->m_cbr_data.startReceiveTime = rxTime;
    reader->m_cbr_data.startTransmitTime = txTime;
    reader->m_cbr_data.currentBusyTime = deltaBusyTime;
    reader->m_cbr_data.currentRxTime = deltaReceiveTime;
    reader->m_cbr_data.currentTxTime = deltaTransmitTime;

    return NL_SKIP;
}