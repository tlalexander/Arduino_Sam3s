// Copyright (c) 2010, Peter Barrett
/*
** Permission to use, copy, modify, and/or distribute this software for
** any purpose with or without fee is hereby granted, provided that the
** above copyright notice and this permission notice appear in all copies.
**
** THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
** WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED
** WARRANTIES OF MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR
** BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES
** OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
** WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION,
** ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS
** SOFTWARE.
*/

#ifndef __USBCORE_H__
#define __USBCORE_H__



//! @name UDP Device control endpoint
//! These macros control the endpoints.
//! @{

//! @name UDP Device control endpoint interrupts
//! These macros control the endpoints interrupts.
//! @{
  //! enables the selected endpoint interrupt
#define  udd_enable_endpoint_interrupt(ep)         (UDP->UDP_IER = UDP_IER_EP0INT << (ep))
  //! disables the selected endpoint interrupt
#define  udd_disable_endpoint_interrupt(ep)        (UDP->UDP_IDR = UDP_IDR_EP0INT << (ep))
  //! tests if the selected endpoint interrupt is enabled
#define  Is_udd_endpoint_interrupt_enabled(ep)     (Tst_bits(UDP->UDP_IMR, UDP_IMR_EP0INT << (ep)))
  //! tests if an interrupt is triggered by the selected endpoint
#define  Is_udd_endpoint_interrupt(ep)             (Tst_bits(UDP->UDP_ISR, UDP_ISR_EP0INT << (ep)))
  //! returns the lowest endpoint number generating an endpoint interrupt or UDD_MAX_PEP_NB if none
#define  udd_get_interrupt_endpoint_number()       (ctz(((UDP->UDP_ISR)  & (UDP->UDP_IMR)) | (1 << UDD_MAX_PEP_NB)))

//! @}


//! \ingroup udd_group
//! \defgroup udd_udp_group USB Device Port Driver
//! UDP low-level driver for USB device mode
//!
//! @{

//! @name UDP Device properties
//! These macros give IP properties (from datasheets)
//! @{
  //! Get maximal number of endpoints (3S 4S 4E, 0~7)
#define  udd_get_endpoint_max_nbr()            (7)
#define  UDD_MAX_PEP_NB                        (udd_get_endpoint_max_nbr()+1)
  //! Get maximal number of banks of endpoint (3S, 1~2)
#define  udd_get_endpoint_bank_max_nbr(ep)     (((ep)==0||(ep)==3)?1:2)
  //! Get maximal size of endpoint (3S, 512/64)
#define  udd_get_endpoint_size_max(ep)         (((ep)==4||(ep)==5)?512:64)
  //! Get isochronous support (3S, endpoints 0 and 3 not support)
#define  udd_is_endpoint_support_iso(ep)       (((ep)==0||(ep)==3)?false:true)
  //! Get DMA support of endpoints (3S, always false)
#define  Is_udd_endpoint_dma_supported(ep)     (false)
  //! Get High Band Width support of endpoints (3S, always false)
#define  Is_udd_endpoint_high_bw_supported(ep) (false)
//! @}

//! @name UDP Device speeds management
//! UDP only supports Full speed.
//! @{
//! @}

//! @name UDP Device vbus pin management
//! UDP peripheral does not support vbus management and it's monitored by a PIO
//! pin.
//! This feature is optional, and it is enabled if USB_VBUS_PIN is defined in
//! board.h and CONF_BOARD_USB_VBUS_DETECT defined in conf_board.h.
//!
//! @note ioport_init() must be invoked before using vbus pin functions since
//!       they use IOPORT API, \see ioport_group.
//!
//! @{
#define UDD_VBUS_DETECT (defined(CONF_BOARD_USB_PORT) && \
 		defined(CONF_BOARD_USB_VBUS_DETECT))
#define UDD_VBUS_IO     (defined(USB_VBUS_PIN) && UDD_VBUS_DETECT)
#ifndef USB_VBUS_WKUP
#  define USB_VBUS_WKUP 0
#endif

#define udd_vbus_init(handler) io_pin_init(USB_VBUS_PIN, USB_VBUS_FLAGS, \
	USB_VBUS_PIN_IRQn, UDD_USB_INT_LEVEL, handler, USB_VBUS_WKUP)
#define Is_udd_vbus_high()           ioport_get_pin_level(USB_VBUS_PIN)
#define Is_udd_vbus_low()            (!Is_udd_vbus_high())
#define udd_enable_vbus_interrupt()  pio_enable_pin_interrupt(USB_VBUS_PIN)
#define udd_disable_vbus_interrupt() pio_disable_pin_interrupt(USB_VBUS_PIN)
#define udd_ack_vbus_interrupt(high) ioport_set_pin_sense_mode(USB_VBUS_PIN,\
	high ? IOPORT_SENSE_LEVEL_LOW : IOPORT_SENSE_LEVEL_HIGH)
//! @}

//! @name UDP peripheral enable/disable
//!
//! @warning The UDP peripheral clock in the Power Management Controller (PMC)
//!          must be enabled before any read/write operations to the UDP registers
//!          including the UDP_TXVC register.
//!
//! @{
#define  udd_enable_periph_ck()              pmc_enable_periph_clk(ID_UDP)
#define  udd_disable_periph_ck()             pmc_disable_periph_clk(ID_UDP)
#define  Is_udd_periph_ck_enabled()          pmc_is_periph_clk_enabled(ID_UDP)
//! @}

//! @name UDP device attach control (by Pull-up)
//! These macros manage the UDP Device attach.
//! @{
  //! detaches from USB bus
#define  udd_detach_device()                 ( Clr_bits(UDP->UDP_TXVC, UDP_TXVC_PUON))
  //! attaches to USB bus
#define  udd_attach_device()                 ( Set_bits(UDP->UDP_TXVC, UDP_TXVC_PUON))
  //! test if the device is detached
#define  Is_udd_detached()                   (!Tst_bits(UDP->UDP_TXVC, UDP_TXVC_PUON))
//! @}

//! @name USBB UDP transceiver management
//! These macros allows to enable/disable pad and USBB hardware
//! @{
#define  udd_enable_transceiver()            ( Clr_bits(UDP->UDP_TXVC, UDP_TXVC_TXVDIS))
#define  udd_disable_transceiver()           ( Set_bits(UDP->UDP_TXVC, UDP_TXVC_TXVDIS))
#define  Is_udd_transceiver_enabled()        (!Tst_bits(UDP->UDP_TXVC, UDP_TXVC_TXVDIS))
//! @}

//! @name UDP device bus events control
//! These macros manage the UDP Device bus events.
//! @{

//! Manage misc events
//! @{
#define  udd_disable_all_events()            (UDP->IDR = 0xFFFF)
#define  udd_disable_endpoint_events()       (UDP->IDR = 0xFF)

#define  udd_enable_wakeups()                (UDP->UDP_IER = (UDP_IER_RXRSM|UDP_IER_EXTRSM|UDP_IER_WAKEUP))
#define  udd_disable_wakeups()               (UDP->UDP_IDR = (UDP_IDR_RXRSM|UDP_IDR_EXTRSM|UDP_IDR_WAKEUP))
#define  udd_ack_wakeups()                   (UDP->UDP_ICR = (UDP_ICR_RXRSM|UDP_ICR_EXTRSM|UDP_ICR_WAKEUP))
#define  Is_udd_any_wakeup()                 (Tst_bits(UDP->UDP_ISR, (UDP_ICR_RXRSM|UDP_ICR_EXTRSM|UDP_ICR_WAKEUP)))
#define  Is_udd_expected_wakeup()            ((UDP->UDP_ISR & (UDP_ICR_RXRSM|UDP_ICR_EXTRSM|UDP_ICR_WAKEUP)) & UDP->UDP_IMR)
//! @}

//! Manage remote wake-up event
//! @{
#define  udd_enable_remote_wake_up()         ( Set_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_RMWUPE))
#define  udd_disable_remote_wake_up()        ( Clr_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_RMWUPE))
#define  Is_udd_remote_wake_up_enabled()     ( Tst_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_RMWUPE))
#define  udd_initiate_remote_wake_up()                            \
	do {                                                      \
		Set_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_ESR);\
		Clr_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_ESR);\
	} while(0)
#define  Is_udd_pending_remote_wake_up()           ( Tst_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_RSMINPR))
//! @}

//! Manage end of resume event (ext/rx resume)
//! The USB controller detects a valid "End of Resume" signal initiated by the host
//! or detects a external "Resume" signal.
//! @{
#define  udd_enable_ext_resume_interrupt()         (UDP->UDP_IER = UDP_IER_EXTRSM)
#define  udd_disable_ext_resume_interrupt()        (UDP->UDP_IDR = UDP_IER_EXTRSM)
#define  Is_udd_ext_resume_interrupt_enabled()     (Tst_bits(UDP->UDP_IMR, UDP_IMR_EXTRSM))
#define  udd_ack_ext_resume()                      (UDP->UDP_ICR = UDP_IER_EXTRSM)
#define  Is_udd_ext_resume()                       (Tst_bits(UDP->UDP_ISR, UDP_IER_EXTRSM))

#define  udd_enable_resume_interrupt()             (UDP->UDP_IER = UDP_IER_RXRSM)
#define  udd_disable_resume_interrupt()            (UDP->UDP_IDR = UDP_IDR_RXRSM)
#define  Is_udd_resume_interrupt_enabled()         (Tst_bits(UDP->UDP_IMR, UDP_IMR_RXRSM))
#define  udd_ack_resume()                          (UDP->UDP_ICR = UDP_ICR_RXRSM)
#define  Is_udd_resume()                           (Tst_bits(UDP->UDP_ISR, UDP_ISR_RXRSM))
//! @}

//! Manage wake-up event (=usb line activity)
//! The USB controller is reactivated by a filtered non-idle signal from the lines
//! @{
#define  udd_enable_wake_up_interrupt()            (UDP->UDP_IER = UDP_IER_WAKEUP)
#define  udd_disable_wake_up_interrupt()           (UDP->UDP_IDR = UDP_IDR_WAKEUP)
#define  Is_udd_wake_up_interrupt_enabled()        (Tst_bits(UDP->UDP_IMR, UDP_IMR_WAKEUP))
#define  udd_ack_wake_up()                         (UDP->UDP_ICR = UDP_ICR_WAKEUP)
#define  Is_udd_wake_up()                          (Tst_bits(UDP->UDP_ISR, UDP_ISR_WAKEUP))
//! @}

//! Manage reset event
//! Set when a USB "End of Reset" has been detected
//! @{
#define  udd_ack_reset()                           (UDP->UDP_ICR = UDP_ISR_ENDBUSRES)
#define  Is_udd_reset()                            (Tst_bits(UDP->UDP_ISR, UDP_ISR_ENDBUSRES))
//! @}

//! Manage start of frame event
//! @{
#define  udd_enable_sof_interrupt()                (UDP->UDP_IER = UDP_ISR_SOFINT)
#define  udd_disable_sof_interrupt()               (UDP->UDP_IDR = UDP_ISR_SOFINT)
#define  Is_udd_sof_interrupt_enabled()            (Tst_bits(UDP->UDP_IMR, UDP_ISR_SOFINT))
#define  udd_ack_sof()                             (UDP->UDP_ICR = UDP_ISR_SOFINT)
#define  Is_udd_sof()                              (Tst_bits(UDP->UDP_ISR, UDP_ISR_SOFINT))
#define  udd_frame_number()                        (Rd_bitfield(UDP->UDP_FRM_NUM, UDP_FRM_NUM_FRM_NUM_Msk))
#define  Is_udd_frame_number_crc_error()           (Tst_bits(UDP->UDP_FRM_NUM, UDP_FRM_NUM_FRM_ERR))
//! @}

//! Manage suspend event
//! @{
#define  udd_enable_suspend_interrupt()            (UDP->UDP_IER = UDP_ISR_RXSUSP)
#define  udd_disable_suspend_interrupt()           (UDP->UDP_IDR = UDP_ISR_RXSUSP)
#define  Is_udd_suspend_interrupt_enabled()        (Tst_bits(UDP->UDP_IMR, UDP_ISR_RXSUSP))
#define  udd_ack_suspend()                         (UDP->UDP_ICR = UDP_ISR_RXSUSP)
#define  udd_raise_suspend()                       (UDP->UDP_ISR = UDP_ISR_RXSUSP)
#define  Is_udd_suspend()                          (Tst_bits(UDP->UDP_ISR, UDP_ISR_RXSUSP))
//! @}

//! @}

//! @name UDP device address control
//! These macros manage the UDP Device address.
//! @{
  //! enables USB device address
#define  udd_enable_address()                      (Set_bits(UDP->UDP_FADDR, UDP_FADDR_FEN))
  //! disables USB device address
#define  udd_disable_address()                     (Clr_bits(UDP->UDP_FADDR, UDP_FADDR_FEN))
#define  Is_udd_address_enabled()                  (Tst_bits(UDP->UDP_FADDR, UDP_FADDR_FEN))
  //! configures the USB device address
#define  udd_configure_address(addr)               (Wr_bitfield(UDP->UDP_FADDR, UDP_FADDR_FADD_Msk, addr))
  //! gets the currently configured USB device address
#define  udd_get_configured_address()              (Rd_bitfield(UDP->UDP_FADDR, UDP_FADDR_FADD_Msk))
  //! sets the device in address state
#define udd_enable_address_state()                 (Set_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_FADDEN))
  //! bring back device to the default state
#define udd_disable_address_state()                (Clr_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_FADDEN))
  //! Check if the device is in address state
#define Is_udd_address_state_enabled()             (Tst_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_FADDEN))
//! @}

//! @name UDP device configured control
//! These macros manage the UDP Device configure state.
//! @{
  //! sets USB device in configured state
#define udd_enable_configured_state()              (Set_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_CONFG))
#define udd_disable_configured_state()             (Clr_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_CONFG))
#define Is_udd_configured_state_enabled()          (Tst_bits(UDP->UDP_GLB_STAT, UDP_GLB_STAT_CONFG))
//! @}


//! @name UDP Device endpoint drivers
//! These macros manage the common features of the endpoints.
//! @{

//! Generic macro for CSR register access
//! \note Due to synchronization between MCK and UDPCK, the software application
//!       must wait for the end of the write operation before executing another
//!       write by polling the bits which must be set/cleared.
//! \note In a preemptive environment, set or clear the flag and wait for a time
//!       of 1 UDPCK clock cycle and 1 peripheral clock cycle. However,
//!       RX_DATA_BK0, TXPKTRDY, RX_DATA_BK1 require wait times of 3 UDPCK clock
//!       cycles and 5 peripheral clock cycles before accessing DPR.
//! @{
  //! Bitmap for all status bits in CSR that are not effected by a value 1.
#define UDP_REG_NO_EFFECT_1_ALL (UDP_CSR_RX_DATA_BK0 |\
                                 UDP_CSR_RX_DATA_BK1 |\
                                 UDP_CSR_STALLSENT   |\
                                 UDP_CSR_RXSETUP     |\
                                 UDP_CSR_TXCOMP)
/*! Sets specified bit(s) in the UDP_CSR.
 *  \param ep   Endpoint number.
 *  \param bits Bitmap to set to 1.
 */
#define udp_set_csr(ep, bits)                                    \
	do {                                                       \
		volatile uint32_t reg;                             \
		volatile uint32_t nop_count;                       \
		reg  = UDP->UDP_CSR[ep];                           \
		reg |= UDP_REG_NO_EFFECT_1_ALL;                    \
		reg |= (bits);                                     \
		UDP->UDP_CSR[ep] = reg;                            \
		for (nop_count = 0; nop_count < 20; nop_count ++) {\
			__NOP();                                   \
		}                                                  \
	} while (0)
/*! Clears specified bit(s) in the UDP_CSR.
 *  \param ep   Endpoint number.
 *  \param bits Bitmap to set to 0.
 */
#define udp_clear_csr(ep, bits)                                  \
	do {                                                       \
		volatile uint32_t reg;                             \
		volatile uint32_t nop_count;                       \
		reg  = UDP->UDP_CSR[ep];                           \
		reg |= UDP_REG_NO_EFFECT_1_ALL;                    \
		reg &= ~(bits);                                    \
		UDP->UDP_CSR[ep] = reg;                            \
		for (nop_count = 0; nop_count < 20; nop_count ++) {\
			__NOP();                                   \
		}                                                  \
	} while (0)
/*! Write specified bit(s) in the UDP_CSR.
 *  \param ep   Endpoint number.
 *  \param bits Bitmap to write.
 */
#define udp_write_csr(ep, mask, bits)                            \
	do {                                                       \
		volatile uint32_t reg;                             \
		volatile uint32_t nop_count;                       \
		reg  = UDP->UDP_CSR[ep];                           \
		reg |= UDP_REG_NO_EFFECT_1_ALL;                    \
		reg &= ~(mask);                                    \
		reg |= bits & mask;                                \
		UDP->UDP_CSR[ep] = reg;                            \
		for (nop_count = 0; nop_count < 20; nop_count ++) {\
			__NOP();                                   \
		}                                                  \
	} while (0);
//! @}

#define  udd_get_endpoint_status(ep)               (UDP->UDP_CSR[ep])

#define  Is_udd_endpoint_status_enabled(status)        (Tst_bits(status, UDP_CSR_EPEDS))
#define  Is_udd_endpoint_status_in(status)             (Tst_bits(status, UDP_CSR_DIR))
#define  Is_udd_endpoint_status_rx_bank0(status)       (Tst_bits(status, UDP_CSR_RX_DATA_BK0))
#define  Is_udd_endpoint_status_rx_bank1(status)       (Tst_bits(status, UDP_CSR_RX_DATA_BK1))
#define  Is_udd_endpoint_status_rx_all(status)         ((status & (UDP_CSR_RX_DATA_BK0|UDP_CSR_RX_DATA_BK1)) == (UDP_CSR_RX_DATA_BK0|UDP_CSR_RX_DATA_BK1))
#define  Is_udd_endpoint_status_rx_any_bank(status)    (Tst_bits(status, UDP_CSR_RX_DATA_BK0|UDP_CSR_RX_DATA_BK0))
#define  Is_udd_endpoint_status_stalled(status)        (Tst_bits(status, UDP_CSR_FORCESTALL))
#define  Is_udd_endpoint_status_stall_sent(status)     (Tst_bits(status, UDP_CSR_STALLSENT))
#define  Is_udd_endpoint_status_stall_pending(status)  (Tst_bits(status, UDP_CSR_STALLSENT|UDP_CSR_FORCESTALL))
#define  Is_udd_endpoint_status_rx_setup(status)       (Tst_bits(status, UDP_CSR_RXSETUP))
#define  Is_udd_endpoint_status_tx_complete(status)    (Tst_bits(status, UDP_CSR_TXCOMP))
#define  Is_udd_endpoint_status_tx_ready(status)       (Tst_bits(status, UDP_CSR_TXPKTRDY))
#define  Is_udd_endpoint_status_error(status)          (Tst_bits(status, UDP_CSR_ISOERROR))

#define  udd_get_endpoint_status_type(status)          (Rd_bits(status, UDP_CSR_EPTYPE_Msk))
#define  udd_get_endpoint_status_toggle(status)        (Rd_bits(status, UDP_CSR_DTGLE))

#define  udd_get_endpoint_status_byte_count(status)    (Rd_bitfield(status, UDP_CSR_RXBYTECNT_Msk))

//! @name UDP Device endpoint configuration
//! @{
  //! enables the selected endpoint
#define  udd_enable_endpoint(ep)                   udp_set_csr(ep, UDP_CSR_EPEDS)
  //! disables the selected endpoint
#define  udd_disable_endpoint(ep)                  (Clr_bits(UDP->UDP_CSR[ep], UDP_CSR_EPEDS))
  //! tests if the selected endpoint is enabled
#define  Is_udd_endpoint_enabled(ep)               (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_EPEDS))
  //! resets the selected endpoint
#define  udd_reset_endpoint(ep)                                   \
	do {                                                      \
		Set_bits(UDP->UDP_RST_EP, UDP_RST_EP_EP0 << (ep));\
		Clr_bits(UDP->UDP_RST_EP, UDP_RST_EP_EP0 << (ep));\
	} while(0)
  //! tests if the selected endpoint is being reset
#define  Is_udd_resetting_endpoint(ep)             (Tst_bits(UDP->UDP_RST_EP, UDP_RST_EP_EP0 << (ep)))

  //! configures the selected endpoint type (shifted)
#define  udd_configure_endpoint_type(ep, type)     udp_write_csr(ep, UDP_CSR_EPTYPE_Msk, type)
  //! gets the configured selected endpoint type (shifted)
#define  udd_get_endpoint_type(ep)                 (Rd_bits(UDP->UDP_CSR[ep], UDP_CSR_EPTYPE_Msk))
#define  Is_udd_endpoint_type_in(ep)               (Tst_bits(UDP->UDP_CSR[ep], 0x4 << UDP_CSR_EPTYPE_Pos))
  //! configures the selected endpoint direction (shifted)
#define  udd_configure_endpoint_direction(ep, dir) udp_write_csr(ep, UDP_CSR_DIR, (dir << 7) & UDP_CSR_DIR)
  //! configures the selected endpoint direction as IN (shifted)
#define  udd_set_endpoint_direction_in(ep)         udp_set_csr(ep, UDP_CSR_DIR)
  //! gets the configured selected endpoint direction (shifted)
#define  udd_get_endpoint_direction(ep)            (Rd_bits(UDP->UDP_CSR[ep], UDP_CSR_DIR))
#define  Is_udd_endpoint_in(ep)                    (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_DIR))

  //! configures selected endpoint in one step (type and dir are not shifted definitions)
#define  udd_configure_endpoint(ep, type, dir)            \
	do {                                              \
		udp_write_csr(ep,                         \
		              (UDP_CSR_EPTYPE_Msk          \
		               |UDP_CSR_DIR|UDP_CSR_EPEDS), \
		              ((((type) << UDP_CSR_EPTYPE_Pos)\
				         & UDP_CSR_EPTYPE_Msk)\
		                |(((dir) << 7) & UDP_CSR_DIR) \
		                |(UDP_CSR_EPEDS)));          \
	} while(0)

  //! tests if current endpoint is configured (=enabled)
#define  Is_udd_endpoint_configured(ep)            (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_EPEDS))
  //! returns the control direction
#define  udd_control_direction()                   (Rd_bits(UDP->UDP_CSR[0], UDP_CSR_DIR))

  //! returns data toggle
#define  udd_data_toggle(ep)                       (Rd_bits(UDP->UDP_CSR[ep], UDP_CSR_DTGLE))
//! @}


//! @name UDP Device control endpoint
//! These macros control the endpoints.
//! @{

//! @name UDP Device control endpoint interrupts
//! These macros control the endpoints interrupts.
//! @{
  //! enables the selected endpoint interrupt
#define  udd_enable_endpoint_interrupt(ep)         (UDP->UDP_IER = UDP_IER_EP0INT << (ep))
  //! disables the selected endpoint interrupt
#define  udd_disable_endpoint_interrupt(ep)        (UDP->UDP_IDR = UDP_IDR_EP0INT << (ep))
  //! tests if the selected endpoint interrupt is enabled
#define  Is_udd_endpoint_interrupt_enabled(ep)     (Tst_bits(UDP->UDP_IMR, UDP_IMR_EP0INT << (ep)))
  //! tests if an interrupt is triggered by the selected endpoint
#define  Is_udd_endpoint_interrupt(ep)             (Tst_bits(UDP->UDP_ISR, UDP_ISR_EP0INT << (ep)))
  //! returns the lowest endpoint number generating an endpoint interrupt or UDD_MAX_PEP_NB if none
#define  udd_get_interrupt_endpoint_number()       (ctz(((UDP->UDP_ISR)  & (UDP->UDP_IMR)) | (1 << UDD_MAX_PEP_NB)))
//! @}

//! @name UDP Device control endpoint errors
//! These macros control the endpoint errors.
//! @{
#define  Is_udd_endpoint_stall_pending(ep)         (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_FORCESTALL|UDP_CSR_STALLSENT))
  //! enables the STALL handshake
#define  udd_enable_stall_handshake(ep)            udp_set_csr(ep, UDP_CSR_FORCESTALL)
  //! disables the STALL handshake
#define  udd_disable_stall_handshake(ep)           udp_clear_csr(ep, UDP_CSR_FORCESTALL)
  //! tests if STALL handshake request is running
#define  Is_udd_endpoint_stall_requested(ep)       (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_FORCESTALL))
  //! tests if STALL sent
#define  Is_udd_stall(ep)                          (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_STALLSENT))
  //! acks STALL sent
#define  udd_ack_stall(ep)                         udp_clear_csr(ep, UDP_CSR_STALLSENT)

  //! tests if CRC ERROR ISO detected
#define  Is_udd_crc_error(ep)                      (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_ISOERROR))
  //! acks CRC ERROR ISO detected
#define  udd_ack_crc_error(ep)                     udp_clear_csr(ep, UDP_CSR_ISOERROR)
//! @}

//! @name UDP Device control endpoint transfer
//! These macros control the endpoint transfer.
//! @{
  //! returns the byte count
#define  udd_byte_count(ep)                        (Rd_bitfield(UDP->UDP_CSR[ep], UDP_CSR_RXBYTECNT_Msk))

  //! test if both banks received
#define  Is_udd_all_banks_received(ep)             ((UDP->UDP_CSR[ep] & (UDP_CSR_RX_DATA_BK0|UDP_CSR_RX_DATA_BK1)) == (UDP_CSR_RX_DATA_BK0|UDP_CSR_RX_DATA_BK1))
  //! test if Any of the Bank received
#define  Is_udd_any_bank_received(ep)              (Tst_bits(UDP->UDP_CSR[ep], (UDP_CSR_RX_DATA_BK0|UDP_CSR_RX_DATA_BK1)))
  //! test if Bank 0 received
#define  Is_udd_bank0_received(ep)                 (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_RX_DATA_BK0))
  //! acks Bank 0 received
#define  udd_ack_bank0_received(ep)                udp_clear_csr(ep, UDP_CSR_RX_DATA_BK0)
  //! test if Bank 1 received
#define  Is_udd_bank1_received(ep)                 (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_RX_DATA_BK1))
  //! acks Bank 1 received
#define  udd_ack_bank1_received(ep)                udp_clear_csr(ep, UDP_CSR_RX_DATA_BK1)
  //! returns the number of received banks
#define  udd_nb_banks_received(ep)                 (Rd_bitfield(UDP->UDP_CSR[ep], UDP_CSR_RX_DATA_BK0) + \
		Rd_bitfield(UDP->UDP_CSR[ep], UDP_CSR_RX_DATA_BK1))

  //! tests if SETUP received
#define  Is_udd_setup_received(ep)                 (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_RXSETUP))
  //! acks SETUP received
#define  udd_ack_setup_received(ep)                udp_clear_csr(ep, UDP_CSR_RXSETUP)

  //! tests if IN pending (TX ready or IN sending)
#define  Is_udd_in_pending(ep)                     (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_TXPKTRDY|UDP_CSR_TXCOMP))
  //! tests if IN sending
#define  Is_udd_in_sent(ep)                        (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_TXCOMP))
  //! acks IN sending
#define  udd_ack_in_sent(ep)                       udp_clear_csr(ep, UDP_CSR_TXCOMP)

  //! tests if transmit packet is ready
#define  Is_udd_transmit_ready(ep)                 (Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_TXPKTRDY))
  //! set transmit packet ready
#define  udd_set_transmit_ready(ep)                udp_set_csr(ep, UDP_CSR_TXPKTRDY)
  //! cancel transmission data held in banks (if TXPKTRDY is set)
#define  udd_kill_data_in_fifo(ep, dual_bank)                                 \
	do {                                                                  \
		if ((dual_bank)) {                                            \
			Clr_bits(UDP->UDP_CSR[ep], UDP_CSR_TXPKTRDY);         \
			while ( Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_TXPKTRDY));\
			Set_bits(UDP->UDP_CSR[ep], UDP_CSR_TXPKTRDY);         \
			while (!Tst_bits(UDP->UDP_CSR[ep], UDP_CSR_TXPKTRDY));\
		}                                                             \
		udp_clear_csr(ep, UDP_CSR_TXPKTRDY);                          \
	} while (0)

  //! read one byte from endpoint fifo
#define  udd_endpoint_fifo_read(ep)                   (UDP->UDP_FDR[ep])
  //! write one byte to endpoint fifo
#define  udd_endpoint_fifo_write(ep, byte) \
	do {                               \
		UDP->UDP_FDR[ep] = byte;   \
	} while (0)

//! @}





//	Standard requests
#define GET_STATUS					0
#define CLEAR_FEATURE				1
#define SET_FEATURE					3
#define SET_ADDRESS					5
#define GET_DESCRIPTOR				6
#define SET_DESCRIPTOR				7
#define GET_CONFIGURATION			8
#define SET_CONFIGURATION			9
#define GET_INTERFACE				10
#define SET_INTERFACE				11


// bmRequestType
#define REQUEST_HOSTTODEVICE		0x00
#define REQUEST_DEVICETOHOST		0x80
#define REQUEST_DIRECTION			0x80

#define REQUEST_STANDARD			0x00
#define REQUEST_CLASS				0x20
#define REQUEST_VENDOR				0x40
#define REQUEST_TYPE				0x60

#define REQUEST_DEVICE				0x00
#define REQUEST_INTERFACE			0x01
#define REQUEST_ENDPOINT			0x02
#define REQUEST_OTHER				0x03
#define REQUEST_RECIPIENT			0x1F

#define REQUEST_DEVICETOHOST_CLASS_INTERFACE  (REQUEST_DEVICETOHOST + REQUEST_CLASS + REQUEST_INTERFACE)
#define REQUEST_HOSTTODEVICE_CLASS_INTERFACE  (REQUEST_HOSTTODEVICE + REQUEST_CLASS + REQUEST_INTERFACE)

//	Class requests

#define CDC_SET_LINE_CODING			0x20
#define CDC_GET_LINE_CODING			0x21
#define CDC_SET_CONTROL_LINE_STATE	0x22

#define MSC_RESET					0xFF
#define MSC_GET_MAX_LUN				0xFE

#define HID_GET_REPORT				0x01
#define HID_GET_IDLE				0x02
#define HID_GET_PROTOCOL			0x03
#define HID_SET_REPORT				0x09
#define HID_SET_IDLE				0x0A
#define HID_SET_PROTOCOL			0x0B

//	Descriptors

#define USB_DEVICE_DESC_SIZE 18
#define USB_CONFIGUARTION_DESC_SIZE 9
#define USB_INTERFACE_DESC_SIZE 9
#define USB_ENDPOINT_DESC_SIZE 7

#define USB_DEVICE_DESCRIPTOR_TYPE             1
#define USB_CONFIGURATION_DESCRIPTOR_TYPE      2
#define USB_STRING_DESCRIPTOR_TYPE             3
#define USB_INTERFACE_DESCRIPTOR_TYPE          4
#define USB_ENDPOINT_DESCRIPTOR_TYPE           5
#define USB_DEVICE_QUALIFIER                   6
#define USB_OTHER_SPEED_CONFIGURATION          7

#define USB_DEVICE_CLASS_COMMUNICATIONS        0x02
#define USB_DEVICE_CLASS_HUMAN_INTERFACE       0x03
#define USB_DEVICE_CLASS_STORAGE               0x08
#define USB_DEVICE_CLASS_VENDOR_SPECIFIC       0xFF

#define USB_CONFIG_POWERED_MASK                0x40
#define USB_CONFIG_BUS_POWERED                 0x80
#define USB_CONFIG_SELF_POWERED                0xC0
#define USB_CONFIG_REMOTE_WAKEUP               0x20

// bMaxPower in Configuration Descriptor
#define USB_CONFIG_POWER_MA(mA)                ((mA)/2)

// bEndpointAddress in Endpoint Descriptor
#define USB_ENDPOINT_DIRECTION_MASK            0x80
#define USB_ENDPOINT_OUT(addr)                 ((addr) | 0x00)
#define USB_ENDPOINT_IN(addr)                  ((addr) | 0x80)

#define USB_ENDPOINT_TYPE_MASK                 0x03
#define USB_ENDPOINT_TYPE_CONTROL              0x00
#define USB_ENDPOINT_TYPE_ISOCHRONOUS          0x01
#define USB_ENDPOINT_TYPE_BULK                 0x02
#define USB_ENDPOINT_TYPE_INTERRUPT            0x03

#define TOBYTES(x) ((x) & 0xFF),(((x) >> 8) & 0xFF)

#define CDC_V1_10                               0x0110
#define CDC_COMMUNICATION_INTERFACE_CLASS       0x02

#define CDC_CALL_MANAGEMENT                     0x01
#define CDC_ABSTRACT_CONTROL_MODEL              0x02
#define CDC_HEADER                              0x00
#define CDC_ABSTRACT_CONTROL_MANAGEMENT         0x02
#define CDC_UNION                               0x06
#define CDC_CS_INTERFACE                        0x24
#define CDC_CS_ENDPOINT                         0x25
#define CDC_DATA_INTERFACE_CLASS                0x0A

#define MSC_SUBCLASS_SCSI						0x06
#define MSC_PROTOCOL_BULK_ONLY					0x50

#define HID_HID_DESCRIPTOR_TYPE					0x21
#define HID_REPORT_DESCRIPTOR_TYPE				0x22
#define HID_PHYSICAL_DESCRIPTOR_TYPE			0x23

_Pragma("pack(1)")

//	Device
typedef struct {
	uint8_t len;				// 18
	uint8_t dtype;				// 1 USB_DEVICE_DESCRIPTOR_TYPE
	uint16_t usbVersion;		// 0x200
	uint8_t	deviceClass;
	uint8_t	deviceSubClass;
	uint8_t	deviceProtocol;
	uint8_t	packetSize0;		// Packet 0
	uint16_t	idVendor;
	uint16_t	idProduct;
	uint16_t	deviceVersion;	// 0x100
	uint8_t	iManufacturer;
	uint8_t	iProduct;
	uint8_t	iSerialNumber;
	uint8_t	bNumConfigurations;
} DeviceDescriptor;

//	Config
typedef struct {
	uint8_t	len;			// 9
	uint8_t	dtype;			// 2
	uint16_t clen;			// total length
	uint8_t	numInterfaces;
	uint8_t	config;
	uint8_t	iconfig;
	uint8_t	attributes;
	uint8_t	maxPower;
} ConfigDescriptor;

//	String

//	Interface
typedef struct
{
	uint8_t len;		// 9
	uint8_t dtype;		// 4
	uint8_t number;
	uint8_t alternate;
	uint8_t numEndpoints;
	uint8_t interfaceClass;
	uint8_t interfaceSubClass;
	uint8_t protocol;
	uint8_t iInterface;
} InterfaceDescriptor;

//	Endpoint
typedef struct
{
	uint8_t len;		// 7
	uint8_t dtype;		// 5
	uint8_t addr;
	uint8_t attr;
	uint16_t packetSize;
	uint8_t interval;
} EndpointDescriptor;

// Interface Association Descriptor
// Used to bind 2 interfaces together in CDC compostite device
typedef struct
{
	uint8_t len;				// 8
	uint8_t dtype;				// 11
	uint8_t firstInterface;
	uint8_t interfaceCount;
	uint8_t functionClass;
	uint8_t funtionSubClass;
	uint8_t functionProtocol;
	uint8_t iInterface;
} IADDescriptor;

//	CDC CS interface descriptor
typedef struct
{
	uint8_t len;		// 5
	uint8_t dtype;		// 0x24
	uint8_t subtype;
	uint8_t d0;
	uint8_t d1;
} CDCCSInterfaceDescriptor;

typedef struct
{
	uint8_t len;		// 4
	uint8_t dtype;		// 0x24
	uint8_t subtype;
	uint8_t d0;
} CDCCSInterfaceDescriptor4;

typedef struct
{
    uint8_t	len;
    uint8_t 	dtype;		// 0x24
    uint8_t 	subtype;	// 1
    uint8_t 	bmCapabilities;
    uint8_t 	bDataInterface;
} CMFunctionalDescriptor;

typedef struct
{
    uint8_t	len;
    uint8_t 	dtype;		// 0x24
    uint8_t 	subtype;	// 1
    uint8_t 	bmCapabilities;
} ACMFunctionalDescriptor;

typedef struct
{
	//	IAD
	IADDescriptor				iad;	// Only needed on compound device

	//	Control
	InterfaceDescriptor			cif;
	CDCCSInterfaceDescriptor	header;
	CMFunctionalDescriptor		callManagement;			// Call Management
	ACMFunctionalDescriptor		controlManagement;		// ACM
	CDCCSInterfaceDescriptor	functionalDescriptor;	// CDC_UNION
	EndpointDescriptor			cifin;

	//	Data
	InterfaceDescriptor			dif;
	EndpointDescriptor			in;
	EndpointDescriptor			out;
} CDCDescriptor;

typedef struct
{
	InterfaceDescriptor			msc;
	EndpointDescriptor			in;
	EndpointDescriptor			out;
} MSCDescriptor;

typedef struct
{
	uint8_t len;			// 9
	uint8_t dtype;			// 0x21
	uint8_t addr;
	uint8_t	versionL;		// 0x101
	uint8_t	versionH;		// 0x101
	uint8_t	country;
	uint8_t	desctype;		// 0x22 report
	uint8_t	descLenL;
	uint8_t	descLenH;
} HIDDescDescriptor;

typedef struct
{
	InterfaceDescriptor		hid;
	HIDDescDescriptor		desc;
	EndpointDescriptor		in;
} HIDDescriptor;

_Pragma("pack()")

#define D_DEVICE(_class,_subClass,_proto,_packetSize0,_vid,_pid,_version,_im,_ip,_is,_configs) \
	{ 18, 1, 0x200, _class,_subClass,_proto,_packetSize0,_vid,_pid,_version,_im,_ip,_is,_configs }

#define D_CONFIG(_totalLength,_interfaces) \
	{ 9, 2, _totalLength,_interfaces, 1, 0, USB_CONFIG_SELF_POWERED, USB_CONFIG_POWER_MA(500) }

#define D_OTHERCONFIG(_totalLength,_interfaces) \
	{ 9, 7, _totalLength,_interfaces, 1, 0, USB_CONFIG_SELF_POWERED, USB_CONFIG_POWER_MA(500) }

#define D_INTERFACE(_n,_numEndpoints,_class,_subClass,_protocol) \
	{ 9, 4, _n, 0, _numEndpoints, _class,_subClass, _protocol, 0 }

#define D_ENDPOINT(_addr,_attr,_packetSize, _interval) \
	{ 7, 5, _addr,_attr,_packetSize, _interval }

#define D_QUALIFIER(_class,_subClass,_proto,_packetSize0,_configs) \
	{ 10, 6, 0x200, _class,_subClass,_proto,_packetSize0,_configs }

#define D_IAD(_firstInterface, _count, _class, _subClass, _protocol) \
	{ 8, 11, _firstInterface, _count, _class, _subClass, _protocol, 0 }

#define D_HIDREPORT(_descriptorLength) \
	{ 9, 0x21, 0x1, 0x1, 0, 1, 0x22, _descriptorLength, 0 }

#define D_CDCCS(_subtype,_d0,_d1)	{ 5, 0x24, _subtype, _d0, _d1 }
#define D_CDCCS4(_subtype,_d0)		{ 4, 0x24, _subtype, _d0 }

#endif
