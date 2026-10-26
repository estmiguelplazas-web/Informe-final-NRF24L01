# nrf24.py  -  Driver del NRF24L01 para MicroPython (Raspberry Pi Pico 2W)
# Cada funcion de configuracion escribe el registro del modulo y se puede
# verificar leyendolo de vuelta con leer_config().
import time

# ---------- Registros (datasheet nRF24L01 v2.0, seccion 9) ----------
CONFIG, EN_AA, EN_RXADDR, SETUP_AW, SETUP_RETR = 0x00, 0x01, 0x02, 0x03, 0x04
RF_CH, RF_SETUP, STATUS, OBSERVE_TX = 0x05, 0x06, 0x07, 0x08
RX_ADDR_P0, TX_ADDR, RX_PW_P0, FIFO_STATUS = 0x0A, 0x10, 0x11, 0x17

# ---------- Comandos SPI ----------
R_RX_PAYLOAD, W_TX_PAYLOAD = 0x61, 0xA0
FLUSH_TX, FLUSH_RX, REUSE_TX_PL, NOP = 0xE1, 0xE2, 0xE3, 0xFF

# ---------- Bits utiles ----------
PWR_UP, PRIM_RX = 0x02, 0x01
EN_CRC, CRCO = 0x08, 0x04          # CRC activado, CRC de 2 bytes
RX_DR, TX_DS, MAX_RT = 0x40, 0x20, 0x10
RF_DR_LOW, RF_DR_HIGH = 0x20, 0x08  # RF_DR_LOW (250k) solo existe en el nRF24L01+

POT_DBM = (-18, -12, -6, 0)        # RF_PWR = 0, 1, 2, 3  (potencia de TRANSMISION)


class NRF24:
    def __init__(self, spi, csn, ce, payload=32, addr=b"UMNG1"):
        self.spi, self.csn, self.ce = spi, csn, ce
        self.payload = payload
        self.ack = True                # auto-ACK + reintentos
        self.crc = True                # CRC de 2 bytes (obligatorio si ack=True)
        self.reintentos = 0            # reintentos del ultimo envio (ARC_CNT)
        self.csn(1)
        self.ce(0)
        time.sleep_ms(100)
        self.write_reg(SETUP_AW, 0x03)         # direcciones de 5 bytes
        self.write_reg(EN_RXADDR, 0x01)        # solo pipe 0
        self.write_reg(TX_ADDR, addr)
        self.write_reg(RX_ADDR_P0, addr)       # misma direccion: necesaria para recibir el ACK
        self.write_reg(RX_PW_P0, payload)      # payload fijo de 32 bytes
        self.set_modo(ack=True, crc=True)
        self.write_reg(RF_SETUP, 0x07)         # 1 Mbps, 0 dBm, LNA alta
        self.write_reg(RF_CH, 40)
        self.limpiar()

    # ================= SPI basico =================
    def cmd(self, c):
        self.csn(0)
        self.spi.write(bytes([c]))
        self.csn(1)

    def write_reg(self, reg, val):
        if isinstance(val, int):
            val = bytes([val])
        self.csn(0)
        self.spi.write(bytes([0x20 | reg]) + val)
        self.csn(1)

    def read_reg(self, reg):
        rx = bytearray(2)
        self.csn(0)
        self.spi.write_readinto(bytes([reg, 0xFF]), rx)
        self.csn(1)
        return rx[1]

    def status(self):
        rx = bytearray(1)
        self.csn(0)
        self.spi.write_readinto(b'\xff', rx)
        self.csn(1)
        return rx[0]

    def presente(self):
        """True si el modulo responde (SETUP_AW se lee como 0x03)."""
        return self.read_reg(SETUP_AW) == 0x03

    def limpiar(self):
        self.cmd(FLUSH_TX)
        self.cmd(FLUSH_RX)
        self.write_reg(STATUS, 0x70)           # borra RX_DR, TX_DS, MAX_RT

    # ================= Configuracion =================
    def _config(self, extra=0):
        return (EN_CRC | CRCO if self.crc else 0) | extra

    def set_modo(self, ack, crc):
        """ack=True : auto-ACK + 15 reintentos (CRC obligatorio).
           ack=False: sin ACK ni reintentos (el CRC se puede apagar: pruebas de BER)."""
        self.ack = ack
        self.crc = crc or ack                  # con auto-ACK el CRC no se puede apagar
        self.write_reg(EN_AA, 0x01 if ack else 0x00)
        self.write_reg(SETUP_RETR, 0x2F if ack else 0x00)   # ARD=750us, ARC=15
        self.write_reg(CONFIG, self._config())

    def set_channel(self, ch):
        # f = 2400 + RF_CH [MHz], RF_CH = 0..125
        self.write_reg(RF_CH, max(0, min(125, int(ch))))

    def set_power(self, nivel):
        """Potencia de TX: 0=-18, 1=-12, 2=-6, 3=0 dBm (bits RF_PWR = RF_SETUP[2:1])."""
        rf = self.read_reg(RF_SETUP) & ~0x06
        self.write_reg(RF_SETUP, rf | (nivel << 1))

    def set_datarate(self, vel):
        """vel = "250K", "1M" o "2M". RF_DR_LOW/RF_DR_HIGH = RF_SETUP[5] y [3]."""
        rf = self.read_reg(RF_SETUP) & ~(RF_DR_LOW | RF_DR_HIGH)
        rf |= {"250K": RF_DR_LOW, "1M": 0, "2M": RF_DR_HIGH}[vel]
        self.write_reg(RF_SETUP, rf)

    def set_lna(self, alta):
        """LNA_HCURR = RF_SETUP[0]. Baja ganancia: -1.5 dB de sensibilidad."""
        rf = self.read_reg(RF_SETUP) & ~0x01
        self.write_reg(RF_SETUP, rf | (1 if alta else 0))

    def soporta_250k(self):
        """El nRF24L01 original solo llega a 1 y 2 Mbps; el '+' tambien 250 kbps.
        Se prueba escribiendo el bit RF_DR_LOW y leyendolo de vuelta."""
        antes = self.read_reg(RF_SETUP)
        self.write_reg(RF_SETUP, antes | RF_DR_LOW)
        ok = bool(self.read_reg(RF_SETUP) & RF_DR_LOW)
        self.write_reg(RF_SETUP, antes)
        return ok

    def leer_config(self):
        """Lee los registros del modulo y devuelve lo que REALMENTE quedo configurado."""
        rf = self.read_reg(RF_SETUP)
        if rf & RF_DR_LOW:
            vel = "250K"
        elif rf & RF_DR_HIGH:
            vel = "2M"
        else:
            vel = "1M"
        return {"canal": self.read_reg(RF_CH),
                "pot": POT_DBM[(rf >> 1) & 0x03],
                "vel": vel,
                "lna": rf & 0x01,
                "ack": self.read_reg(EN_AA) & 0x01,
                "crc": 1 if self.read_reg(CONFIG) & EN_CRC else 0}

    def aplicar(self, canal, nivel_pot, vel, lna, ack, crc):
        """Escribe toda la configuracion y la verifica leyendo los registros.
        Devuelve True si el modulo quedo como se pidio."""
        self.ce(0)
        self.set_modo(ack, crc)
        self.set_channel(canal)
        self.set_power(nivel_pot)
        self.set_datarate(vel)
        self.set_lna(lna)
        self.limpiar()
        c = self.leer_config()
        return (c["canal"] == canal and c["pot"] == POT_DBM[nivel_pot] and
                c["vel"] == vel and c["lna"] == (1 if lna else 0) and
                c["ack"] == (1 if ack else 0))

    def apagar(self):
        """Power down (consumo minimo) al volver al menu."""
        self.ce(0)
        self.write_reg(CONFIG, self._config())

    # ================= Transmisor =================
    def _modo_tx(self):
        self.ce(0)
        cfg = self._config(PWR_UP)
        if self.read_reg(CONFIG) != cfg:       # solo espera si estaba apagado o en RX
            self.write_reg(CONFIG, cfg)
            time.sleep_ms(2)                   # Tpd2stby = 1.5 ms

    def _cargar(self, data):
        data = bytes(data)[:self.payload]
        data = data + bytes(self.payload - len(data))
        self.cmd(FLUSH_TX)
        self.write_reg(STATUS, 0x70)
        self.csn(0)
        self.spi.write(bytes([W_TX_PAYLOAD]) + data)
        self.csn(1)

    def send(self, data, timeout_ms=300):
        """Envia un paquete.
           ack=True : True si llego el ACK (TX_DS), False si se agotaron los reintentos (MAX_RT).
           ack=False: True cuando termino de transmitir (no se sabe si llego).
           El numero de reintentos queda en self.reintentos."""
        self._modo_tx()
        self._cargar(data)
        self.ce(1)
        time.sleep_us(20)                      # CE alto minimo 10 us
        self.ce(0)
        ok = False
        t0 = time.ticks_ms()
        while time.ticks_diff(time.ticks_ms(), t0) < timeout_ms:
            st = self.status()
            if st & TX_DS:
                ok = True
                break
            if st & MAX_RT:
                break
        self.reintentos = self.read_reg(OBSERVE_TX) & 0x0F     # ARC_CNT
        self.write_reg(STATUS, TX_DS | MAX_RT)
        if not ok:
            self.cmd(FLUSH_TX)
        return ok

    def iniciar_continuo(self, data):
        """Transmision continua (100 % de ciclo de trabajo) para el analizador de
        espectros: se envia un paquete y luego REUSE_TX_PL con CE en alto lo repite
        sin parar. Requiere ack=False."""
        self._modo_tx()
        self._cargar(data)
        self.ce(1)
        t0 = time.ticks_ms()
        while not (self.status() & TX_DS) and time.ticks_diff(time.ticks_ms(), t0) < 20:
            pass
        self.write_reg(STATUS, TX_DS)
        self.cmd(REUSE_TX_PL)                  # CE sigue en alto: repite el paquete

    def detener_continuo(self):
        self.ce(0)
        self.cmd(FLUSH_TX)                     # tambien cancela REUSE_TX_PL
        self.write_reg(STATUS, 0x70)

    # ================= Receptor =================
    def start_rx(self):
        self.ce(0)
        self.write_reg(CONFIG, self._config(PWR_UP | PRIM_RX))
        time.sleep_ms(5)
        self.limpiar()
        self.ce(1)
        time.sleep_us(150)                     # Tstby2a = 130 us

    def available(self):
        return not (self.read_reg(FIFO_STATUS) & 0x01)     # RX FIFO no vacia

    def recv(self):
        """Devuelve los 32 bytes recibidos tal cual (sin quitar ceros)."""
        rx = bytearray(self.payload + 1)
        self.csn(0)
        self.spi.write_readinto(bytes([R_RX_PAYLOAD]) + bytes(self.payload), rx)
        self.csn(1)
        self.write_reg(STATUS, RX_DR)
        return bytes(rx[1:])