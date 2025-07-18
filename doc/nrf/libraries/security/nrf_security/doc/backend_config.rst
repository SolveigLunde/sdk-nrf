.. _nrf_security_backend_config:
.. _nrf_security_legacy_config:
.. _legacy_crypto_support:
.. _nrf_security_drivers_legacy:

Configuring nRF Security with legacy crypto APIs
################################################

.. contents::
   :local:
   :depth: 2

.. legacy_crypto_support_def_start

The legacy crypto is a subsystem for software that requires Mbed TLS crypto toolbox API functions that are prefixed with ``mbedtls_``.
It provides TLS and DTLS support and backwards compatibility with older applications that do not use :ref:`PSA Crypto APIs <psa_crypto_support>`.
The legacy crypto uses *alternative implementations* (called backends) of the drivers that are also used for the PSA Crypto API support.

To enable the legacy crypto support mode of nRF Security:

1. Set the :kconfig:option:`CONFIG_NRF_SECURITY` Kconfig option.
2. :ref:`Configure the legacy crypto backend <nrf_security_legacy_backend_config>`.

.. legacy_crypto_support_def_end

Deprecation of legacy crypto support
************************************

.. caution::
   |legacy_crypto_deprecation_note|

The following changes have been made to the legacy crypto support with the deprecation announcement:

* Enabling the Kconfig option :kconfig:option:`CONFIG_NRF_SECURITY` replaces using the Kconfig option :kconfig:option:`CONFIG_NORDIC_SECURITY_BACKEND` to enable the legacy crypto support.
  Setting :kconfig:option:`CONFIG_NORDIC_SECURITY_BACKEND` also enables :kconfig:option:`CONFIG_MBEDTLS_LEGACY_CRYPTO_C`, which shows a deprecation warning in the build output.
* The legacy Mbed TLS APIs no longer support the glued functionality.
* Legacy configurations no longer have an effect on the configurations for the secure image of a TF-M build.

.. _nrf_security_legacy_backend_config:
.. _nrf_security_backend_config_multiple:
.. _nrf_security_backends_cc3xx:
.. _nrf_security_backends_oberon:

Configuring the legacy crypto backend
*************************************

The legacy crypto backend is a term describing a set of configurations that provide low-level integration with Mbed TLS that were used before the adoption of PSA Crypto APIs in the |NCS|.
These legacy crypto backends are provided as *alternative implementations* of the drivers that are also used for the PSA Crypto API support.

The legacy crypto configuration only allows one backend to be enabled at the same time.

The following table lists the available legacy crypto backends with their respective Kconfig options and the corresponding hardware platforms.

+-----------------------------------------------+-----------------------------------------+----------------------------------------------------------+
|                Driver library                 |          Legacy crypto backend          |               Supported hardware platforms               |
+===============================================+=========================================+==========================================================+
| :ref:`nrf_cc3xx_mbedcrypto_readme`            | :kconfig:option:`CONFIG_CC3XX_BACKEND`  | nRF52840, nRF5340, nRF91 Series devices                  |
+-----------------------------------------------+-----------------------------------------+----------------------------------------------------------+
| :ref:`nrf_oberon <nrfxlib:nrf_oberon_readme>` | :kconfig:option:`CONFIG_OBERON_BACKEND` | nRF devices with Arm Cortex®-M0, -M4, or -M33 processors |
+-----------------------------------------------+-----------------------------------------+----------------------------------------------------------+

.. note::
   Enabling the CryptoCell by using :kconfig:option:`CONFIG_CC3XX_BACKEND` in a non-secure image of a TF-M build will have no effect.

AES configuration
*****************

The AES core is enabled with the Kconfig option :kconfig:option:`CONFIG_MBEDTLS_AES_C`.

This enables AES ECB cipher mode and allows the following ciphers and modes to be configured:

* CTR
* CBC
* XTS
* CMAC
* CCM/CCM*
* GCM

Feature support
===============

+-------------+-------------------+-------------+
| Cipher mode | Backend           | Key size    |
+=============+===================+=============+
| ECB         | nrf_cc310         | 128-bit key |
|             +-------------------+-------------+
|             | nrf_cc312         | 128-bit key |
|             |                   +-------------+
|             |                   | 192-bit key |
|             |                   +-------------+
|             |                   | 256-bit key |
|             +-------------------+-------------+
|             | nrf_oberon        | 128-bit key |
|             |                   +-------------+
|             |                   | 192-bit key |
|             |                   +-------------+
|             |                   | 256-bit key |
+-------------+-------------------+-------------+

.. note::
   The :ref:`nrf_security_backends_oberon` uses some functionality from the original Mbed TLS for AES operations.

.. note::
   |original_mbedtls_def_note|

AES cipher configuration
************************

To configure AES cipher modes, set the following Kconfig options:

+--------------+----------------------------------------------------+----------------------------------------+
| Cipher mode  | Configurations                                     | Note                                   |
+==============+====================================================+========================================+
| CTR          | :kconfig:option:`CONFIG_MBEDTLS_CIPHER_MODE_CTR`   |                                        |
+--------------+----------------------------------------------------+----------------------------------------+
| CBC          | :kconfig:option:`CONFIG_MBEDTLS_CIPHER_MODE_CBC`   |                                        |
+--------------+----------------------------------------------------+----------------------------------------+
| XTS          | :kconfig:option:`CONFIG_MBEDTLS_CIPHER_MODE_XTS`   | nrf_oberon only                        |
+--------------+----------------------------------------------------+----------------------------------------+

.. note::
   AES cipher modes are dependent on enabling AES core support according to `AES configuration`_.

Feature support
===============

+-------------+-------------------+-------------+-----------------------+
| Cipher mode | Backend           | Key size    | Note                  |
+=============+===================+=============+=======================+
| CTR         | nrf_cc310         | 128-bit key |                       |
|             +-------------------+-------------+-----------------------+
|             | nrf_cc312         | 128-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 192-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 256-bit key |                       |
|             +-------------------+-------------+-----------------------+
|             | nrf_oberon        | 128-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 192-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 256-bit key |                       |
+-------------+-------------------+-------------+-----------------------+
| CBC         | nrf_cc310         | 128-bit key |                       |
|             +-------------------+-------------+-----------------------+
|             | nrf_cc312         | 128-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 192-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 256-bit key |                       |
|             +-------------------+-------------+-----------------------+
|             | nrf_oberon        | 128-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 192-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 256-bit key |                       |
+-------------+-------------------+-------------+-----------------------+
| XTS         | nrf_cc310         | N/A         | Backend not supported |
|             +-------------------+-------------+-----------------------+
|             | nrf_cc312         | N/A         | Backend not supported |
|             +-------------------+-------------+-----------------------+
|             | nrf_oberon        | 128-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 192-bit key |                       |
|             |                   +-------------+-----------------------+
|             |                   | 256-bit key |                       |
+-------------+-------------------+-------------+-----------------------+

CMAC configuration
******************

To configure Cipher-based Message Authentication Code (CMAC) support, set the :kconfig:option:`CONFIG_MBEDTLS_CMAC_C` Kconfig option.

Feature support
===============

+-----------+-------------------+-------------+
| Algorithm | Backend           | Key size    |
+===========+===================+=============+
| CMAC      | nrf_cc310         | 128-bit key |
|           +-------------------+-------------+
|           | nrf_cc312         | 128-bit key |
|           |                   +-------------+
|           |                   | 192-bit key |
|           |                   +-------------+
|           |                   | 256-bit key |
|           +-------------------+-------------+
|           | nrf_oberon        | 128-bit key |
|           |                   +-------------+
|           |                   | 192-bit key |
|           |                   +-------------+
|           |                   | 256-bit key |
+-----------+-------------------+-------------+


AEAD configurations
*******************

To configure Authenticated Encryption with Associated Data (AEAD), set the following Kconfig options:

+--------------+------------------------------------------------+-----------------------------------------+
| AEAD cipher  | Configurations                                 | Note                                    |
+==============+================================================+=========================================+
| AES CCM/CCM* | :kconfig:option:`CONFIG_MBEDTLS_CCM_C`         |                                         |
+--------------+------------------------------------------------+-----------------------------------------+
| AES GCM      | :kconfig:option:`CONFIG_MBEDTLS_GCM_C`         | nrf_oberon or nrf_cc312                 |
+--------------+------------------------------------------------+-----------------------------------------+
| ChaCha20     | :kconfig:option:`CONFIG_MBEDTLS_CHACHA20_C`    |                                         |
+--------------+------------------------------------------------+-----------------------------------------+
| Poly1305     | :kconfig:option:`CONFIG_MBEDTLS_POLY1305_C`    |                                         |
+--------------+------------------------------------------------+-----------------------------------------+
| ChaCha-Poly  | :kconfig:option:`CONFIG_MBEDTLS_CHACHAPOLY_C`  | Requires `Poly1305` and `ChaCha20`      |
+--------------+------------------------------------------------+-----------------------------------------+

.. note::
   * AEAD AES cipher modes are dependent on enabling AES core support according to `AES configuration`_.
   * When the ``nrf_cc310`` backend is used, AES GCM is provided by the original Mbed TLS implementation.
   * The ChaCha-Poly implemented by the ``nrf_cc3xx`` backend does not support incremental operations.
   * The ChaCha-Poly implemented by the :ref:`nrf_security_backends_cc3xx` does not support incremental operations.

Feature support
===============

+--------------+-------------------+-------------+----------------------------------------------------------------------+
| AEAD cipher  | Backend           | Key size    | Note                                                                 |
+==============+===================+=============+======================================================================+
| AES CCM/CCM* | nrf_cc310         | 128-bit key |                                                                      |
|              +-------------------+-------------+----------------------------------------------------------------------+
|              | nrf_cc312         | 128-bit key |                                                                      |
|              |                   +-------------+----------------------------------------------------------------------+
|              |                   | 192-bit key |                                                                      |
|              |                   +-------------+----------------------------------------------------------------------+
|              |                   | 256-bit key |                                                                      |
|              +-------------------+-------------+----------------------------------------------------------------------+
|              | nrf_oberon        | 128-bit key |                                                                      |
|              |                   +-------------+----------------------------------------------------------------------+
|              |                   | 192-bit key |                                                                      |
|              |                   +-------------+----------------------------------------------------------------------+
|              |                   | 256-bit key |                                                                      |
+--------------+-------------------+-------------+----------------------------------------------------------------------+
| AES GCM      | nrf_cc312         | 128-bit key |                                                                      |
|              |                   +-------------+----------------------------------------------------------------------+
|              |                   | 192-bit key |                                                                      |
|              |                   +-------------+----------------------------------------------------------------------+
|              |                   | 256-bit key |                                                                      |
|              +-------------------+-------------+----------------------------------------------------------------------+
|              | nrf_oberon        | 128-bit key |                                                                      |
|              |                   +-------------+----------------------------------------------------------------------+
|              |                   | 192-bit key |                                                                      |
|              |                   +-------------+----------------------------------------------------------------------+
|              |                   | 256-bit key |                                                                      |
+--------------+-------------------+-------------+----------------------------------------------------------------------+
| ChaCha20     | nrf_cc3xx         | 256-bit key |                                                                      |
|              +-------------------+-------------+----------------------------------------------------------------------+
|              | nrf_oberon        | 256-bit key |                                                                      |
+--------------+-------------------+-------------+----------------------------------------------------------------------+
| Poly1305     | nrf_cc3xx         | 256-bit key |                                                                      |
|              +-------------------+-------------+----------------------------------------------------------------------+
|              | nrf_oberon        | 256-bit key |                                                                      |
+--------------+-------------------+-------------+----------------------------------------------------------------------+
| ChaCha-Poly  | nrf_cc3xx         | 256-bit key | The ChaCha-Poly implementation in :ref:`nrf_security_backends_cc3xx` |
|              |                   |             | does not support incremental operations.                             |
|              +-------------------+-------------+----------------------------------------------------------------------+
|              | nrf_oberon        | 256-bit key |                                                                      |
+--------------+-------------------+-------------+----------------------------------------------------------------------+

DHM configurations
******************

To configure Diffie-Hellman-Merkle (DHM) support, set the :kconfig:option:`CONFIG_MBEDTLS_DHM_C` Kconfig option.

Feature support
===============

+-----------+-------------------+----------------------+-----------------------+
| Algorithm | Backend           | Key size             | Note                  |
+===========+===================+======================+=======================+
| DHM       | nrf_cc3xx          | Limited to 2048 bits |                      |
|           +-------------------+----------------------+-----------------------+
|           | nrf_oberon        | N/A                  | Backend not supported |
+-----------+-------------------+----------------------+-----------------------+

.. note::
   The :ref:`nrf_security_backends_oberon` uses functionality from the original Mbed TLS for DHM operations.

ECC configurations
******************

Elliptic Curve Cryptography (ECC) configuration provides support for Elliptic Curve over GF(p).

To configure ECC core support, set the :kconfig:option:`CONFIG_MBEDTLS_ECP_C` Kconfig option.

Enabling :kconfig:option:`CONFIG_MBEDTLS_ECP_C` will activate configuration options that depend on ECC, such as ECDH, ECDSA, ECJPAKE, and a selection of ECC curves to support in the system.

Feature support
===============

+-----------+-------------------+-------------+------------+
| Algorithm | Backend           | Curve group | Curve type |
+===========+===================+=============+============+
| ECP       | nrf_cc3xx         | NIST        | secp192r1  |
|           |                   |             +------------+
|           |                   |             | secp224r1  |
|           |                   |             +------------+
|           |                   |             | secp256r1  |
|           |                   |             +------------+
|           |                   |             | secp384r1  |
|           |                   |             +------------+
|           |                   |             | secp521r1  |
|           |                   +-------------+------------+
|           |                   | Koblitz     | secp192k1  |
|           |                   |             +------------+
|           |                   |             | secp224k1  |
|           |                   |             +------------+
|           |                   |             | secp256k1  |
|           |                   +-------------+------------+
|           |                   | Curve25519  | Curve25519 |
|           +-------------------+-------------+------------+
|           | nrf_oberon        | NIST        | secp256r1  |
|           |                   |             +------------+
|           |                   |             | secp224r1  |
|           |                   +-------------+------------+
|           |                   | Curve25519  | Curve25519 |
+-----------+-------------------+-------------+------------+

ECDH configurations
*******************

To configure Elliptic Curve Diffie-Hellman (ECDH) support, set the :kconfig:option:`CONFIG_MBEDTLS_ECDH_C` Kconfig option.

+--------------+---------------------------------------------+
| Algorithm    | Configurations                              |
+==============+=============================================+
| ECDH         | :kconfig:option:`CONFIG_MBEDTLS_ECDH_C`     |
+--------------+---------------------------------------------+

.. note::
   * ECDH support depends on `ECC Configurations`_ being enabled.
   * The :ref:`nrf_cc3xx_mbedcrypto_readme` does not integrate on ECP layer.
     Only the top-level APIs for ECDH are replaced.

Feature support
===============

+-----------+-------------------+-------------+------------+
| Algorithm | Backend           | Curve group | Curve type |
+===========+===================+=============+============+
| ECDH      | nrf_cc3xx         | NIST        | secp192r1  |
|           |                   |             +------------+
|           |                   |             | secp224r1  |
|           |                   |             +------------+
|           |                   |             | secp256r1  |
|           |                   |             +------------+
|           |                   |             | secp384r1  |
|           |                   |             +------------+
|           |                   |             | secp521r1  |
|           |                   +-------------+------------+
|           |                   | Koblitz     | secp192k1  |
|           |                   |             +------------+
|           |                   |             | secp224k1  |
|           |                   |             +------------+
|           |                   |             | secp256k1  |
|           |                   +-------------+------------+
|           |                   | Curve25519  | Curve25519 |
|           +-------------------+-------------+------------+
|           | nrf_oberon        | NIST        | secp256r1  |
|           |                   |             +------------+
|           |                   |             | secp224r1  |
|           |                   +-------------+------------+
|           |                   | Curve25519  | Curve25519 |
+-----------+-------------------+-------------+------------+

ECDSA configurations
********************

To configure Elliptic Curve Digital Signature Algorithm (ECDSA) support, set the :kconfig:option:`CONFIG_MBEDTLS_ECDSA_C` Kconfig option.

+--------------+----------------------------------------------+
| Algorithm    | Configurations                               |
+==============+==============================================+
| ECDSA        | :kconfig:option:`CONFIG_MBEDTLS_ECDSA_C`     |
+--------------+----------------------------------------------+

.. note::
   * ECDSA support depends on `ECC Configurations`_ being enabled.
   * The :ref:`nrf_cc3xx_mbedcrypto_readme` does not integrate on ECP layer.
     Only the top-level APIs for ECDSA are replaced.

Feature support
===============

+-----------+-------------------+-------------+------------+
| Algorithm | Backend           | Curve group | Curve type |
+===========+===================+=============+============+
| ECDSA     | nrf_cc3xx         | NIST        | secp192r1  |
|           |                   |             +------------+
|           |                   |             | secp224r1  |
|           |                   |             +------------+
|           |                   |             | secp256r1  |
|           |                   |             +------------+
|           |                   |             | secp384r1  |
|           |                   |             +------------+
|           |                   |             | secp521r1  |
|           |                   +-------------+------------+
|           |                   | Koblitz     | secp192k1  |
|           |                   |             +------------+
|           |                   |             | secp224k1  |
|           |                   |             +------------+
|           |                   |             | secp256k1  |
|           |                   +-------------+------------+
|           |                   | Curve25519  | Curve25519 |
|           +-------------------+-------------+------------+
|           | nrf_oberon        | NIST        | secp256r1  |
|           |                   |             +------------+
|           |                   |             | secp224r1  |
|           |                   +-------------+------------+
|           |                   | Curve25519  | Curve25519 |
+-----------+-------------------+-------------+------------+

ECJPAKE configurations
**********************

To configure Elliptic Curve, Password Authenticated Key Exchange by Juggling (ECJPAKE) support, set the :kconfig:option:`CONFIG_MBEDTLS_ECJPAKE_C` Kconfig option.

+--------------+----------------------------------------------+
| Algorithm    | Configurations                               |
+==============+==============================================+
| ECJPAKE      | :kconfig:option:`CONFIG_MBEDTLS_ECJPAKE_C`   |
+--------------+----------------------------------------------+

.. note::
   ECJPAKE support depends upon `ECC Configurations`_ being enabled.

Feature support
===============

+-----------+-------------------+-------------+------------+
| Algorithm | Backend           | Curve group | Curve type |
+===========+===================+=============+============+
| ECJPAKE   | nrf_cc3xx         | NIST        | secp256r1  |
|           +-------------------+-------------+------------+
|           | nrf_oberon        | NIST        | secp256r1  |
+-----------+-------------------+-------------+------------+


.. _nrf_security_backend_config_ecc_curves:

ECC curves configurations
*************************

It is possible to configure the curves that should be supported in the system depending on the backend selected.

The following curves can be enabled:

+-----------------------------+------------------------------------------------------------+--------------------------+
| Curve                       | Configurations                                             | Note                     |
+=============================+============================================================+==========================+
| NIST secp192r1              | :kconfig:option:`CONFIG_MBEDTLS_ECP_DP_SECP192R1_ENABLED`  |                          |
+-----------------------------+------------------------------------------------------------+--------------------------+
| NIST secp224r1              | :kconfig:option:`CONFIG_MBEDTLS_ECP_DP_SECP224R1_ENABLED`  |                          |
+-----------------------------+------------------------------------------------------------+--------------------------+
| NIST secp256r1              | :kconfig:option:`CONFIG_MBEDTLS_ECP_DP_SECP256R1_ENABLED`  |                          |
+-----------------------------+------------------------------------------------------------+--------------------------+
| NIST secp384r1              | :kconfig:option:`CONFIG_MBEDTLS_ECP_DP_SECP384R1_ENABLED`  |                          |
+-----------------------------+------------------------------------------------------------+--------------------------+
| NIST secp521r1              | :kconfig:option:`CONFIG_MBEDTLS_ECP_DP_SECP521R1_ENABLED`  |                          |
+-----------------------------+------------------------------------------------------------+--------------------------+
| Koblitz secp192k1           | :kconfig:option:`CONFIG_MBEDTLS_ECP_DP_SECP192K1_ENABLED`  |                          |
+-----------------------------+------------------------------------------------------------+--------------------------+
| Koblitz secp224k1           | :kconfig:option:`CONFIG_MBEDTLS_ECP_DP_SECP224K1_ENABLED`  |                          |
+-----------------------------+------------------------------------------------------------+--------------------------+
| Koblitz secp256k1           | :kconfig:option:`CONFIG_MBEDTLS_ECP_DP_SECP256K1_ENABLED`  |                          |
+-----------------------------+------------------------------------------------------------+--------------------------+
| Curve25519                  | :kconfig:option:`CONFIG_MBEDTLS_ECP_DP_CURVE25519_ENABLED` |                          |
+-----------------------------+------------------------------------------------------------+--------------------------+

.. note::
   * The :ref:`nrf_oberon_readme` only supports ECC curve secp224r1 and secp256r1.
   * Choosing the nrf_oberon backend does not allow enabling the rest of the ECC curve types.


RSA configurations
******************

To configure Rivest-Shamir-Adleman (RSA) support, set the :kconfig:option:`CONFIG_MBEDTLS_RSA_C` Kconfig option.

Feature support
===============

+-----------+-------------------+--------------+
| Algorithm | Backend           | Key size     |
+===========+===================+==============+
| RSA       | nrf_cc310         | 1024-bit key |
|           |                   +--------------+
|           |                   | 1536-bit key |
|           |                   +--------------+
|           |                   | 2048-bit key |
|           +-------------------+--------------+
|           | nrf_cc312         | 1024-bit key |
|           |                   +--------------+
|           |                   | 1536-bit key |
|           |                   +--------------+
|           |                   | 2048-bit key |
|           |                   +--------------+
|           |                   | 3072-bit key |
|           +-------------------+--------------+
|           | nrf_oberon        | 1024-bit key |
|           |                   +--------------+
|           |                   | 1536-bit key |
|           |                   +--------------+
|           |                   | 2048-bit key |
|           |                   +--------------+
|           |                   | 3072-bit key |
+-----------+-------------------+--------------+

.. note::
   The :ref:`nrf_security_backends_oberon` uses functionality from the original Mbed TLS for RSA operations.

Secure Hash configurations
**************************

To configure the Secure Hash algorithms, set the following Kconfig options:

+--------------+--------------------+---------------------------------------------+
| Algorithm    | Support            | Backend selection                           |
+==============+====================+=============================================+
| SHA-1        |                    | :kconfig:option:`CONFIG_MBEDTLS_SHA1_C`     |
+--------------+--------------------+---------------------------------------------+
| SHA-224      |                    | :kconfig:option:`CONFIG_MBEDTLS_SHA224_C`   |
+--------------+--------------------+---------------------------------------------+
| SHA-256      |                    | :kconfig:option:`CONFIG_MBEDTLS_SHA256_C`   |
+--------------+--------------------+---------------------------------------------+
| SHA-384      |                    | :kconfig:option:`CONFIG_MBEDTLS_SHA384_C`   |
+--------------+--------------------+---------------------------------------------+
| SHA-512      |                    | :kconfig:option:`CONFIG_MBEDTLS_SHA512_C`   |
+--------------+--------------------+---------------------------------------------+

Feature support
===============

+-----------+--------------------+----------------------------------------+
| Algorithm | Supported backends | Note                                   |
+===========+====================+========================================+
| SHA-1     | nrf_cc3xx          |                                        |
|           +--------------------+                                        |
|           | nrf_oberon         |                                        |
+-----------+--------------------+----------------------------------------+
| SHA-224   | nrf_cc3xx          | SHA-224 must be enabled when enabling  |
|           +--------------------+ SHA-256                                |
|           | nrf_oberon         |                                        |
+-----------+--------------------+----------------------------------------+
| SHA-256   | nrf_cc3xx          |                                        |
|           +--------------------+                                        |
|           | nrf_oberon         |                                        |
+-----------+--------------------+----------------------------------------+
| SHA-384   | nrf_cc3xx          |                                        |
|           +--------------------+                                        |
|           | nrf_oberon         |                                        |
+-----------+--------------------+----------------------------------------+
| SHA-512   | nrf_cc3xx          |                                        |
|           +--------------------+                                        |
|           | nrf_oberon         |                                        |
+-----------+--------------------+----------------------------------------+
