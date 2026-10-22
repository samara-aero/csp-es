#include <check.h>
#include <pthread.h>
#include <signal.h>
#include <string.h>
#include <unistd.h>
#include "../include/csp/csp.h"
#include "../include/csp/interfaces/csp_if_can.h"
#include "../src/csp_qfifo.h"

/* Every frame of a CAN packet carries one CFP counter value, and receivers
 * reassemble by that value, so packets sent from several threads at once must
 * leave the interface whole, one after another. */

#define THREADS 8
#define PACKETS 50
/* 38 bytes is 6 frames in both CFP 1 and CFP 2 */
#define LENGTH 38
#define FRAMES 6
/* Time each frame takes to "send"; wide enough for other senders to cut in */
#define FRAME_US 20

#define SRC 5
#define DST 7

struct frame {
	int thread;
	uint32_t id;
	uint8_t dlc;
	uint8_t data[8];
};

static struct frame frames[THREADS * PACKETS * FRAMES];
static int n_frames;
static pthread_mutex_t frames_lock = PTHREAD_MUTEX_INITIALIZER;
static __thread int this_thread;

static csp_iface_t tx_iface;
static csp_can_interface_data_t tx_ifdata;

static int record_frame(void * driver_data, uint32_t id, const uint8_t * data, uint8_t dlc) {
	(void)driver_data;
	pthread_mutex_lock(&frames_lock);
	ck_assert_int_lt(n_frames, (int)(sizeof(frames) / sizeof(frames[0])));
	struct frame * f = &frames[n_frames++];
	f->thread = this_thread;
	f->id = id;
	f->dlc = dlc;
	memcpy(f->data, data, dlc);
	pthread_mutex_unlock(&frames_lock);
	usleep(FRAME_US);
	return CSP_ERR_NONE;
}

static void add_can_interface(uint8_t version, csp_can_driver_tx_t tx_func) {
	csp_conf.version = version;
	csp_init();
	n_frames = 0;
	memset(&tx_iface, 0, sizeof(tx_iface));
	memset(&tx_ifdata, 0, sizeof(tx_ifdata));
	tx_ifdata.tx_func = tx_func;
	tx_iface.name = "CAN_TEST";
	tx_iface.addr = SRC;
	tx_iface.interface_data = &tx_ifdata;
	ck_assert_int_eq(csp_can_add_interface(&tx_iface), CSP_ERR_NONE);
}

/* Bytes 0 and 1 name the packet; the rest are a pattern unique to it. */
static uint8_t payload_byte(int thread, int seq, int i) {
	if (i == 0) {
		return (uint8_t)thread;
	}
	if (i == 1) {
		return (uint8_t)seq;
	}
	return (uint8_t)(thread * 31 + seq * 7 + i);
}

static void send_packet(int thread, int seq) {
	csp_packet_t * packet = csp_buffer_get(0);
	ck_assert_ptr_nonnull(packet);
	packet->id.pri = CSP_PRIO_NORM;
	packet->id.src = SRC;
	packet->id.dst = DST;
	packet->id.dport = 10;
	packet->id.sport = 20;
	packet->length = LENGTH;
	for (int i = 0; i < LENGTH; i++) {
		packet->data[i] = payload_byte(thread, seq, i);
	}
	ck_assert_int_eq(tx_iface.nexthop(&tx_iface, CSP_NO_VIA_ADDRESS, packet, 1), CSP_ERR_NONE);
}

static void * send_packets(void * arg) {
	this_thread = (int)(intptr_t)arg;
	for (int seq = 0; seq < PACKETS; seq++) {
		send_packet(this_thread, seq);
	}
	return NULL;
}

static uint32_t frame_counter(uint32_t id) {
	return csp_conf.version == 1 ? CFP_ID(id) : (id >> CFP2_SC_OFFSET) & CFP2_SC_MASK;
}

static uint32_t counter_mask(void) {
	return csp_conf.version == 1 ? (1U << CFP_ID_SIZE) - 1 : CFP2_SC_MASK;
}

static bool last_frame(uint32_t id) {
	return csp_conf.version == 1 ? CFP_REMAIN(id) == 0 : (id >> CFP2_END_OFFSET) & CFP2_END_MASK;
}

/* Frames n*FRAMES .. n*FRAMES+FRAMES-1 must be packet n: one sender, one counter value, the next one in turn. */
static void check_packets_are_whole(void) {
	ck_assert_int_eq(n_frames, THREADS * PACKETS * FRAMES);
	for (int p = 0; p < THREADS * PACKETS; p++) {
		const struct frame * first = &frames[p * FRAMES];
		ck_assert_uint_eq(frame_counter(first->id), (uint32_t)p & counter_mask());
		for (int f = 1; f < FRAMES; f++) {
			const struct frame * next = &frames[p * FRAMES + f];
			ck_assert_msg(next->thread == first->thread,
						  "packet %d: frame %d is from thread %d, its first frame from thread %d",
						  p, f, next->thread, first->thread);
			ck_assert_uint_eq(frame_counter(next->id), frame_counter(first->id));
		}
	}
}

/* Feeds every frame to a receiving CAN interface: each packet must arrive once, intact. */
static void check_receiver_gets_every_packet(void) {
	static csp_iface_t rx_iface;
	static csp_can_interface_data_t rx_ifdata;
	memset(&rx_iface, 0, sizeof(rx_iface));
	memset(&rx_ifdata, 0, sizeof(rx_ifdata));
	rx_iface.name = "CAN_RX";
	rx_iface.addr = DST;
	rx_iface.interface_data = &rx_ifdata;

	bool seen[THREADS][PACKETS] = {{false}};
	int received = 0;
	for (int i = 0; i < n_frames; i++) {
		ck_assert_int_eq(csp_can_rx(&rx_iface, frames[i].id, frames[i].data, frames[i].dlc, NULL), CSP_ERR_NONE);
		if (!last_frame(frames[i].id)) {
			continue;
		}
		csp_qfifo_t input;
		ck_assert_int_eq(csp_qfifo_read(&input), CSP_ERR_NONE);
		csp_packet_t * packet = input.packet;
		ck_assert_uint_eq(packet->length, LENGTH);
		int thread = packet->data[0], seq = packet->data[1];
		ck_assert_int_eq(thread, frames[i].thread);
		ck_assert_int_lt(seq, PACKETS);
		ck_assert_msg(!seen[thread][seq], "thread %d packet %d received twice", thread, seq);
		for (int b = 0; b < LENGTH; b++) {
			ck_assert_uint_eq(packet->data[b], payload_byte(thread, seq, b));
		}
		seen[thread][seq] = true;
		received++;
		csp_buffer_free(packet);
	}
	ck_assert_int_eq(received, THREADS * PACKETS);
}

static void send_from_threads_at_once(void) {
	pthread_t threads[THREADS];
	for (int t = 0; t < THREADS; t++) {
		ck_assert_int_eq(pthread_create(&threads[t], NULL, send_packets, (void *)(intptr_t)t), 0);
	}
	for (int t = 0; t < THREADS; t++) {
		pthread_join(threads[t], NULL);
	}
}

START_TEST(test_can2_concurrent_packets_stay_whole)
{
	add_can_interface(2, record_frame);
	send_from_threads_at_once();
	check_packets_are_whole();
	check_receiver_gets_every_packet();
}
END_TEST

START_TEST(test_can1_concurrent_packets_stay_whole)
{
	add_can_interface(1, record_frame);
	send_from_threads_at_once();
	check_packets_are_whole();
	check_receiver_gets_every_packet();
}
END_TEST

/* --- A signal must not let a waiting sender cut in --------------------------- */

static pthread_mutex_t gate_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t gate_cond = PTHREAD_COND_INITIALIZER;
static bool holder_sending, holder_released;

/* Thread 0's first frame waits until the test releases it, holding the packet open. */
static int gated_frame(void * driver_data, uint32_t id, const uint8_t * data, uint8_t dlc) {
	if (this_thread == 0) {
		pthread_mutex_lock(&gate_lock);
		if (!holder_sending) {
			holder_sending = true;
			pthread_cond_broadcast(&gate_cond);
			while (!holder_released) {
				pthread_cond_wait(&gate_cond, &gate_lock);
			}
		}
		pthread_mutex_unlock(&gate_lock);
	}
	return record_frame(driver_data, id, data, dlc);
}

static void * send_one(void * arg) {
	this_thread = (int)(intptr_t)arg;
	send_packet(this_thread, 0);
	return NULL;
}

static void on_signal(int sig) {
	(void)sig;
}

START_TEST(test_can_signal_does_not_let_a_waiting_sender_in)
{
	/* No SA_RESTART: a blocking wait in the waiting sender returns EINTR */
	struct sigaction sa = {.sa_handler = on_signal};
	ck_assert_int_eq(sigaction(SIGUSR1, &sa, NULL), 0);
	add_can_interface(2, gated_frame);

	pthread_t holder, waiter;
	ck_assert_int_eq(pthread_create(&holder, NULL, send_one, (void *)(intptr_t)0), 0);
	pthread_mutex_lock(&gate_lock);
	while (!holder_sending) {
		pthread_cond_wait(&gate_cond, &gate_lock);
	}
	pthread_mutex_unlock(&gate_lock);

	ck_assert_int_eq(pthread_create(&waiter, NULL, send_one, (void *)(intptr_t)1), 0);
	for (int i = 0; i < 5; i++) {
		usleep(10000);
		pthread_kill(waiter, SIGUSR1);
	}
	usleep(10000);
	pthread_mutex_lock(&frames_lock);
	ck_assert_msg(n_frames == 0, "the waiting sender sent %d frames while another packet was being sent", n_frames);
	pthread_mutex_unlock(&frames_lock);

	pthread_mutex_lock(&gate_lock);
	holder_released = true;
	pthread_cond_broadcast(&gate_cond);
	pthread_mutex_unlock(&gate_lock);
	pthread_join(holder, NULL);
	pthread_join(waiter, NULL);

	ck_assert_int_eq(n_frames, 2 * FRAMES);
	for (int f = 0; f < 2 * FRAMES; f++) {
		ck_assert_int_eq(frames[f].thread, f < FRAMES ? 0 : 1);
	}
}
END_TEST

Suite * can_tx_suite(void)
{
	Suite * s = suite_create("CAN TX");
	TCase * tc = tcase_create("concurrent senders");
	tcase_add_test(tc, test_can2_concurrent_packets_stay_whole);
	tcase_add_test(tc, test_can1_concurrent_packets_stay_whole);
	tcase_add_test(tc, test_can_signal_does_not_let_a_waiting_sender_in);
	suite_add_tcase(s, tc);
	return s;
}
