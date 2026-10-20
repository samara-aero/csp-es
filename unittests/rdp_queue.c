#include <check.h>
#include "../include/csp/csp.h"
#include "../src/csp_conn.h"
#include "../src/csp_rdp_queue.h"

/* The RDP TX/RX queues are shared by all connections. Looking up one
 * connection's packets must not change the owner of the others, or their
 * retransmissions are lost (or sent on the wrong connection). */
START_TEST(test_rdp_queue_get_keeps_owner)
{
	csp_conn_t conn_a = {0}, conn_b = {0};

	csp_init();
	csp_rdp_queue_flush(NULL);

	csp_packet_t * pa = csp_buffer_get(0);
	csp_packet_t * pb = csp_buffer_get(0);
	ck_assert_ptr_nonnull(pa);
	ck_assert_ptr_nonnull(pb);

	csp_rdp_queue_tx_add(&conn_a, pa);
	csp_rdp_queue_tx_add(&conn_b, pb);

	/* Scanning for B passes over A's packet */
	ck_assert_ptr_eq(csp_rdp_queue_tx_get(&conn_b), pb);
	ck_assert_ptr_eq(pa->conn, &conn_a);
	ck_assert_ptr_eq(csp_rdp_queue_tx_get(&conn_a), pa);
	ck_assert_ptr_null(csp_rdp_queue_tx_get(&conn_b));

	csp_buffer_free(pa);
	csp_buffer_free(pb);
}
END_TEST

Suite * rdp_queue_suite(void)
{
	Suite * s = suite_create("RDP queue");
	TCase * tc = tcase_create("owner");
	tcase_add_test(tc, test_rdp_queue_get_keeps_owner);
	suite_add_tcase(s, tc);
	return s;
}
