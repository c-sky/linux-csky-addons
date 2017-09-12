obj-$(CONFIG_CRYPTO_DEV_CSKY) += csky-cipher.o

ifeq ($(CONFIG_CSKY_CRYPTO_AES), y)
csky-cipher-objs += csky_aes.o
endif

ifeq ($(CONFIG_CSKY_CRYPTO_TDES), y)
csky-cipher-objs += csky_tdes.o
endif

ifeq ($(CONFIG_CSKY_CRYPTO_CRC), y)
csky-cipher-objs += csky_crc.o
endif

ifeq ($(CONFIG_CSKY_CRYPTO_RSA), y)
csky-cipher-objs += csky_rsa.o
endif

ifeq ($(CONFIG_CSKY_CRYPTO_SHA), y)
csky-cipher-objs += csky_sha.o
endif
