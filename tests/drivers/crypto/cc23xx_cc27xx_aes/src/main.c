/*
 * Copyright (c) 2026 Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Known-answer regression tests for the TI CC23XX/CC27XX AES accelerator
 * driver (drivers/crypto/crypto_cc23xx_cc27xx.c), run over the Zephyr
 * crypto API. Runs in both PIO and DMA driver configurations (see
 * testcase.yaml).
 */

#include <zephyr/ztest.h>
#include <zephyr/device.h>
#include <zephyr/crypto/crypto.h>

#include <string.h>

static const struct device *const dev = DEVICE_DT_GET_ONE(ti_cc23xx_cc27xx_aes);

#define CAP_FLAGS (CAP_RAW_KEY | CAP_SYNC_OPS | CAP_SEPARATE_IO_BUFS)

/* SP800-38A / FIPS-197 AES-128 key */
static const uint8_t kat_key[16] = {
	0x2b, 0x7e, 0x15, 0x16, 0x28, 0xae, 0xd2, 0xa6,
	0xab, 0xf7, 0x15, 0x88, 0x09, 0xcf, 0x4f, 0x3c,
};

/* SP800-38A 64-byte test plaintext */
static const uint8_t kat_plaintext[64] = {
	0x6b, 0xc1, 0xbe, 0xe2, 0x2e, 0x40, 0x9f, 0x96,
	0xe9, 0x3d, 0x7e, 0x11, 0x73, 0x93, 0x17, 0x2a,
	0xae, 0x2d, 0x8a, 0x57, 0x1e, 0x03, 0xac, 0x9c,
	0x9e, 0xb7, 0x6f, 0xac, 0x45, 0xaf, 0x8e, 0x51,
	0x30, 0xc8, 0x1c, 0x46, 0xa3, 0x5c, 0xe4, 0x11,
	0xe5, 0xfb, 0xc1, 0x19, 0x1a, 0x0a, 0x52, 0xef,
	0xf6, 0x9f, 0x24, 0x45, 0xdf, 0x4f, 0x9b, 0x17,
	0xad, 0x2b, 0x41, 0x7b, 0xe6, 0x6c, 0x37, 0x10,
};

/* SP800-38A F.1.1 ECB-AES128 ciphertext for kat_plaintext */
static const uint8_t ecb_ciphertext[64] = {
	0x3a, 0xd7, 0x7b, 0xb4, 0x0d, 0x7a, 0x36, 0x60,
	0xa8, 0x9e, 0xca, 0xf3, 0x24, 0x66, 0xef, 0x97,
	0xf5, 0xd3, 0xd5, 0x85, 0x03, 0xb9, 0x69, 0x9d,
	0xe7, 0x85, 0x89, 0x5a, 0x96, 0xfd, 0xba, 0xaf,
	0x43, 0xb1, 0xcd, 0x7f, 0x59, 0x8e, 0xce, 0x23,
	0x88, 0x1b, 0x00, 0xe3, 0xed, 0x03, 0x06, 0x88,
	0x7b, 0x0c, 0x78, 0x5e, 0x27, 0xe8, 0xad, 0x3f,
	0x82, 0x23, 0x20, 0x71, 0x04, 0x72, 0x5d, 0xd4,
};

/*
 * AES-CTR with 96-bit nonce f0f1..fb and 32-bit counter starting at 0
 * (Zephyr crypto API convention, same vector as samples/drivers/crypto).
 */
static const uint8_t ctr_iv[12] = {
	0xf0, 0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7,
	0xf8, 0xf9, 0xfa, 0xfb,
};

static const uint8_t ctr_ciphertext[64] = {
	0x22, 0xe5, 0x2f, 0xb1, 0x77, 0xd8, 0x65, 0xb2,
	0xf7, 0xc6, 0xb5, 0x12, 0x69, 0x2d, 0x11, 0x4d,
	0xed, 0x6c, 0x1c, 0x72, 0x25, 0xda, 0xf6, 0xa2,
	0xaa, 0xd9, 0xd3, 0xda, 0x2d, 0xba, 0x21, 0x68,
	0x35, 0xc0, 0xaf, 0x6b, 0x6f, 0x40, 0xc3, 0xc6,
	0xef, 0xc5, 0x85, 0xd0, 0x90, 0x2c, 0xc2, 0x63,
	0x12, 0x2b, 0xc5, 0x8e, 0x72, 0xde, 0x5c, 0xa2,
	0xa3, 0x5c, 0x85, 0x3a, 0xb9, 0x2c, 0x06, 0xbb,
};

/* RFC 3610 test vector #1 */
static const uint8_t ccm_key[16] = {
	0xc0, 0xc1, 0xc2, 0xc3, 0xc4, 0xc5, 0xc6, 0xc7,
	0xc8, 0xc9, 0xca, 0xcb, 0xcc, 0xcd, 0xce, 0xcf,
};

static const uint8_t ccm_nonce[13] = {
	0x00, 0x00, 0x00, 0x03, 0x02, 0x01, 0x00, 0xa0,
	0xa1, 0xa2, 0xa3, 0xa4, 0xa5,
};

static const uint8_t ccm_hdr[8] = {
	0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
};

static const uint8_t ccm_data[23] = {
	0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	0x10, 0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17,
	0x18, 0x19, 0x1a, 0x1b, 0x1c, 0x1d, 0x1e,
};

/* 23 bytes of ciphertext followed by the 8-byte tag */
static const uint8_t ccm_expected[31] = {
	0x58, 0x8c, 0x97, 0x9a, 0x61, 0xc6, 0x63, 0xd2,
	0xf0, 0x66, 0xd0, 0xc2, 0xc0, 0xf9, 0x89, 0x80,
	0x6d, 0x5f, 0x6b, 0x61, 0xda, 0xc3, 0x84, 0x17,
	0xe8, 0xd1, 0x2c, 0xfd, 0xf9, 0x26, 0xe0,
};

static int ecb_encrypt(uint8_t *in, int in_len, uint8_t *out, int out_max)
{
	struct cipher_ctx ctx = {
		.keylen = sizeof(kat_key),
		.key.bit_stream = (uint8_t *)kat_key,
		.flags = CAP_FLAGS,
	};
	struct cipher_pkt pkt = {
		.in_buf = in,
		.in_len = in_len,
		.out_buf = out,
		.out_buf_max = out_max,
	};
	int ret;

	zassert_ok(cipher_begin_session(dev, &ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_ENCRYPT),
		   "ECB session setup failed");

	ret = cipher_block_op(&ctx, &pkt);

	cipher_free_session(dev, &ctx);

	return ret;
}

static int ctr_op(enum cipher_op op, uint8_t *in, int in_len, uint8_t *out, int out_max)
{
	struct cipher_ctx ctx = {
		.keylen = sizeof(kat_key),
		.key.bit_stream = (uint8_t *)kat_key,
		.flags = CAP_FLAGS,
		.mode_params.ctr_info.ctr_len = 32,
	};
	struct cipher_pkt pkt = {
		.in_buf = in,
		.in_len = in_len,
		.out_buf = out,
		.out_buf_max = out_max,
	};
	int ret;

	zassert_ok(cipher_begin_session(dev, &ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CTR, op),
		   "CTR session setup failed");

	ret = cipher_ctr_op(&ctx, &pkt, (uint8_t *)ctr_iv);

	cipher_free_session(dev, &ctx);

	return ret;
}

static int ccm_op(enum cipher_op op, uint8_t *in, int in_len, uint8_t *out, int out_max,
		  uint8_t *tag)
{
	struct cipher_ctx ctx = {
		.keylen = sizeof(ccm_key),
		.key.bit_stream = (uint8_t *)ccm_key,
		.flags = CAP_FLAGS,
		.mode_params.ccm_info = {
			.nonce_len = sizeof(ccm_nonce),
			.tag_len = 8,
		},
	};
	struct cipher_pkt pkt = {
		.in_buf = in,
		.in_len = in_len,
		.out_buf = out,
		.out_buf_max = out_max,
	};
	struct cipher_aead_pkt aead = {
		.ad = (uint8_t *)ccm_hdr,
		.ad_len = sizeof(ccm_hdr),
		.pkt = &pkt,
		.tag = tag,
	};
	int ret;

	zassert_ok(cipher_begin_session(dev, &ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_CCM, op),
		   "CCM session setup failed");

	ret = cipher_ccm_op(&ctx, &aead, (uint8_t *)ccm_nonce);

	cipher_free_session(dev, &ctx);

	return ret;
}

ZTEST(crypto_lpf3_aes, test_ecb_single_block_kat)
{
	/* FIPS-197 appendix C.1 */
	static const uint8_t key[16] = {
		0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07,
		0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d, 0x0e, 0x0f,
	};
	static uint8_t pt[16] = {
		0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
		0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff,
	};
	static const uint8_t ct[16] = {
		0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30,
		0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4, 0xc5, 0x5a,
	};
	uint8_t out[16] = {0};
	struct cipher_ctx ctx = {
		.keylen = sizeof(key),
		.key.bit_stream = (uint8_t *)key,
		.flags = CAP_FLAGS,
	};
	struct cipher_pkt pkt = {
		.in_buf = pt,
		.in_len = sizeof(pt),
		.out_buf = out,
		.out_buf_max = sizeof(out),
	};

	zassert_ok(cipher_begin_session(dev, &ctx, CRYPTO_CIPHER_ALGO_AES,
					CRYPTO_CIPHER_MODE_ECB, CRYPTO_CIPHER_OP_ENCRYPT));
	zassert_ok(cipher_block_op(&ctx, &pkt), "ECB encrypt failed");
	cipher_free_session(dev, &ctx);

	zassert_equal(pkt.out_len, sizeof(ct), "bad out_len %d", pkt.out_len);
	zassert_mem_equal(out, ct, sizeof(ct), "ECB single block mismatch");
}

ZTEST(crypto_lpf3_aes, test_ecb_multi_block_kat)
{
	uint8_t out[64] = {0};

	zassert_ok(ecb_encrypt((uint8_t *)kat_plaintext, sizeof(kat_plaintext), out,
			       sizeof(out)),
		   "ECB encrypt failed");
	zassert_mem_equal(out, ecb_ciphertext, sizeof(ecb_ciphertext),
			  "ECB multi block mismatch");
}

/*
 * The AES input/output DMA channels are independent: each channel's
 * transfer width must be derived from the alignment of the buffer that
 * channel actually accesses (TRM 15.3.7 requires data aligned to the
 * DMA data size). Regression test for the output buffer being less
 * aligned than the input buffer.
 */
ZTEST(crypto_lpf3_aes, test_ecb_unaligned_out)
{
	static uint8_t out_raw[32 + 8];
	uint8_t *out = &out_raw[1];

	memset(out_raw, 0xa5, sizeof(out_raw));

	zassert_ok(ecb_encrypt((uint8_t *)kat_plaintext, 32, out, 32),
		   "ECB encrypt failed");
	zassert_mem_equal(out, ecb_ciphertext, 32, "ECB unaligned-out mismatch");
	zassert_equal(out_raw[0], 0xa5, "byte before out_buf clobbered");
	for (size_t i = 1 + 32; i < sizeof(out_raw); i++) {
		zassert_equal(out_raw[i], 0xa5, "byte after out_buf clobbered");
	}
}

ZTEST(crypto_lpf3_aes, test_ctr_unaligned_out)
{
	static uint8_t out_raw[32 + 8];
	uint8_t *out = &out_raw[1];

	memset(out_raw, 0xa5, sizeof(out_raw));

	zassert_ok(ctr_op(CRYPTO_CIPHER_OP_ENCRYPT, (uint8_t *)kat_plaintext, 32, out, 32),
		   "CTR encrypt failed");
	zassert_mem_equal(out, ctr_ciphertext, 32, "CTR unaligned-out mismatch");
	zassert_equal(out_raw[0], 0xa5, "byte before out_buf clobbered");
	for (size_t i = 1 + 32; i < sizeof(out_raw); i++) {
		zassert_equal(out_raw[i], 0xa5, "byte after out_buf clobbered");
	}
}

ZTEST(crypto_lpf3_aes, test_ctr_kat)
{
	uint8_t out[64] = {0};
	uint8_t back[64] = {0};

	zassert_ok(ctr_op(CRYPTO_CIPHER_OP_ENCRYPT, (uint8_t *)kat_plaintext,
			  sizeof(kat_plaintext), out, sizeof(out)),
		   "CTR encrypt failed");
	zassert_mem_equal(out, ctr_ciphertext, sizeof(ctr_ciphertext), "CTR mismatch");

	zassert_ok(ctr_op(CRYPTO_CIPHER_OP_DECRYPT, out, sizeof(out), back, sizeof(back)),
		   "CTR decrypt failed");
	zassert_mem_equal(back, kat_plaintext, sizeof(kat_plaintext),
			  "CTR roundtrip mismatch");
}

ZTEST(crypto_lpf3_aes, test_ctr_partial_block)
{
	/* 20 bytes: one full block plus a 4-byte tail (CTR is a stream cipher,
	 * so the expected output is a prefix of the full-length vector).
	 */
	uint8_t out[20] = {0};
	uint8_t back[20] = {0};

	zassert_ok(ctr_op(CRYPTO_CIPHER_OP_ENCRYPT, (uint8_t *)kat_plaintext, sizeof(out),
			  out, sizeof(out)),
		   "CTR partial encrypt failed");
	zassert_mem_equal(out, ctr_ciphertext, sizeof(out), "CTR partial mismatch");

	zassert_ok(ctr_op(CRYPTO_CIPHER_OP_DECRYPT, out, sizeof(out), back, sizeof(back)),
		   "CTR partial decrypt failed");
	zassert_mem_equal(back, kat_plaintext, sizeof(back), "CTR partial roundtrip mismatch");
}

ZTEST(crypto_lpf3_aes, test_ccm_kat)
{
	uint8_t out[23] = {0};
	uint8_t tag[8] = {0};
	uint8_t back[23] = {0};

	zassert_ok(ccm_op(CRYPTO_CIPHER_OP_ENCRYPT, (uint8_t *)ccm_data, sizeof(ccm_data),
			  out, sizeof(out), tag),
		   "CCM encrypt failed");
	zassert_mem_equal(out, ccm_expected, sizeof(ccm_data), "CCM ciphertext mismatch");
	zassert_mem_equal(tag, &ccm_expected[sizeof(ccm_data)], sizeof(tag),
			  "CCM tag mismatch");

	zassert_ok(ccm_op(CRYPTO_CIPHER_OP_DECRYPT, out, sizeof(out), back, sizeof(back), tag),
		   "CCM decrypt failed");
	zassert_mem_equal(back, ccm_data, sizeof(ccm_data), "CCM roundtrip mismatch");
}

ZTEST(crypto_lpf3_aes, test_ccm_tag_mismatch)
{
	uint8_t out[23] = {0};
	uint8_t tag[8] = {0};
	uint8_t back[23] = {0};

	zassert_ok(ccm_op(CRYPTO_CIPHER_OP_ENCRYPT, (uint8_t *)ccm_data, sizeof(ccm_data),
			  out, sizeof(out), tag),
		   "CCM encrypt failed");

	tag[0] ^= 0x01;

	zassert_not_equal(ccm_op(CRYPTO_CIPHER_OP_DECRYPT, out, sizeof(out), back,
				 sizeof(back), tag),
			  0, "corrupted tag must be rejected");

	for (size_t i = 0; i < sizeof(back); i++) {
		zassert_equal(back[i], 0, "output must be zeroed on auth failure");
	}
}

static void *crypto_setup(void)
{
	zassert_true(device_is_ready(dev), "crypto device not ready");

	uint32_t flags = crypto_query_hwcaps(dev);

	zassert_equal(flags & CAP_FLAGS, CAP_FLAGS, "missing capabilities");

	return NULL;
}

ZTEST_SUITE(crypto_lpf3_aes, NULL, crypto_setup, NULL, NULL, NULL);
