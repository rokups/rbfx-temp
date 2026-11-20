add_library(ssl STATIC
    ech/ech_helper.c ech/ech_internal.c ech/ech_ssl_apis.c ech/ech_store.c
    bio_ssl.c d1_lib.c d1_msg.c d1_srtp.c d1_transcript.c dgram_demux.c dtls_conn_lookup.c dtls_record_rx.c methods.c pqueue.c priority_queue.c s3_enc.c s3_lib.c s3_msg.c ssl_asn1.c ssl_cert.c ssl_cert_comp.c ssl_ciph.c ssl_conf.c ssl_err_legacy.c ssl_init.c ssl_lib.c ssl_mcnf.c ssl_rsa.c ssl_rsa_legacy.c ssl_sess.c ssl_stat.c ssl_txt.c ssl_utst.c t1_enc.c t1_lib.c t1_trce.c tls13_enc.c tls_depr.c tls_srp.c
    quic/cc_newreno.c quic/json_enc.c quic/qlog.c quic/qlog_event_helpers.c quic/quic_ackm.c quic/quic_cfq.c quic/quic_channel.c quic/quic_demux.c quic/quic_engine.c quic/quic_fc.c quic/quic_fifd.c quic/quic_impl.c quic/quic_lcidm.c quic/quic_method.c quic/quic_obj.c quic/quic_port.c quic/quic_rcidm.c quic/quic_reactor.c quic/quic_reactor_wait_ctx.c quic/quic_record_rx.c quic/quic_record_shared.c quic/quic_record_tx.c quic/quic_record_util.c quic/quic_rstream.c quic/quic_rx_depack.c quic/quic_srt_gen.c quic/quic_srtm.c quic/quic_sstream.c quic/quic_statm.c quic/quic_stream_map.c quic/quic_strm_reas.c quic/quic_thread_assist.c quic/quic_tls.c quic/quic_tls_api.c quic/quic_trace.c quic/quic_tserver.c quic/quic_txp.c quic/quic_txpim.c quic/quic_types.c quic/quic_wire.c quic/quic_wire_pkt.c quic/uint_set.c
    record/rec_layer_d1.c record/rec_layer_s3.c
    record/methods/dtls_meth.c record/methods/tls13_meth.c record/methods/tls1_meth.c record/methods/tls_common.c record/methods/tls_multib.c record/methods/tlsany_meth.c
    rio/poll_builder.c rio/poll_immediate.c rio/rio_notifier.c
    statem/extensions.c statem/extensions_clnt.c statem/extensions_cust.c statem/extensions_srvr.c statem/statem.c statem/statem_clnt.c statem/statem_dtls.c statem/statem_lib.c statem/statem_srvr.c
)
target_include_directories(ssl PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/..)
target_link_libraries(ssl PUBLIC crypto)
if (WIN32)
    target_compile_definitions(ssl PRIVATE WIN32_LEAN_AND_MEAN UNICODE _UNICODE _CRT_SECURE_NO_DEPRECATE _WINSOCK_DEPRECATED_NO_WARNINGS)
endif ()
