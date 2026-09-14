# AZAK Rocket 2025 Payload Firmware
## AZAK Roket 2025 Görev Yükü Yazılımı

This repository contains the payload firmware developed for the AZAK Rocket Team's 2025 rocket project.

Bu repository, AZAK Roket Takımı'nın 2025 yılı roket projesi kapsamında geliştirilen görev yükü yazılımını içermektedir.

---

## Overview | Genel Bakış

The firmware was developed for an embedded payload system and includes real-time task management, sensor acquisition, telemetry packet preparation, and serial communication.

Yazılım; gömülü görev yükü sistemi için geliştirilmiş olup gerçek zamanlı görev yönetimi, sensör verisi toplama, telemetri paketi hazırlama ve seri haberleşme işlemlerini içermektedir.

---

## Main Features | Temel Özellikler

- FreeRTOS-based task structure  
  FreeRTOS tabanlı görev yapısı

- Sensor data acquisition  
  Sensör verilerinin okunması

- Telemetry packet preparation  
  Telemetri paketlerinin hazırlanması

- Serial communication  
  Seri haberleşme

- Byte-level data packing using `memcpy`  
  `memcpy` kullanılarak byte seviyesinde veri paketleme

- Checksum-based packet verification  
  Checksum tabanlı paket doğrulama

- Periodic real-time task execution  
  Periyodik gerçek zamanlı görev çalıştırma

---

## Software Architecture | Yazılım Mimarisi

The firmware is organized using FreeRTOS tasks so that different system operations can execute independently at defined intervals.

Yazılım, farklı sistem işlemlerinin belirlenen zaman aralıklarında bağımsız şekilde çalışabilmesi için FreeRTOS görevleri kullanılarak yapılandırılmıştır.

The main software responsibilities include:

- Sensor acquisition
- Data processing
- Telemetry packet construction
- Communication
- Periodic task scheduling

---

## Telemetry Packetization | Telemetri Paketleme

Telemetry values are converted into a byte-oriented packet structure before transmission.

Telemetri değerleri gönderim öncesinde byte tabanlı bir paket yapısına dönüştürülmektedir.

The firmware uses `memcpy` operations to place variables into the telemetry buffer.

Yazılımda değişkenlerin telemetri buffer'ına aktarılması için `memcpy` işlemleri kullanılmaktadır.

This approach allows different numerical data types to be transferred in a structured binary packet.

Bu yöntem sayesinde farklı sayısal veri tipleri düzenli bir binary paket içerisinde aktarılabilmektedir.

---

## FreeRTOS | Gerçek Zamanlı Görev Yönetimi

FreeRTOS is used to separate system responsibilities into independent tasks.

FreeRTOS, sistem sorumluluklarını bağımsız görevler halinde ayırmak amacıyla kullanılmaktadır.

This structure improves timing control and prevents individual operations from blocking the entire firmware flow.

Bu yapı, zamanlama kontrolünü geliştirir ve tek bir işlemin tüm yazılım akışını bloke etmesini önler.

---

## Technologies | Kullanılan Teknolojiler

- Arduino / Embedded C++
- FreeRTOS
- Serial Communication
- Binary Telemetry
- `memcpy`
- Embedded Systems

---

## Project File | Proje Dosyası

```text
_payload_verici.ino
```

---

## Contributions | Katkılar

This firmware was developed collaboratively as part of the AZAK Rocket Team 2025 project.

Bu yazılım, AZAK Roket Takımı 2025 projesi kapsamında ekip çalışmasıyla geliştirilmiştir.

### Elifsena Aycan

Contributions:

- Communication architecture
- Serial data transmission
- FreeRTOS task structure
- Real-time task organization
- Telemetry packet preparation
- Binary data packing using `memcpy`
- Packet transmission logic

Katkılar:

- Haberleşme mimarisi
- Seri veri iletimi
- FreeRTOS görev yapısı
- Gerçek zamanlı görev organizasyonu
- Telemetri paketlerinin hazırlanması
- `memcpy` kullanılarak binary veri paketleme
- Paket gönderim mantığı

### Ecem Nur Yıldız

Contributions:

- Sensor data acquisition
- Sensor-related processing
- Sensor measurements and associated calculations

Katkılar:

- Sensörlerden veri okunması
- Sensör verilerinin işlenmesi
- Sensör ölçümleri ve ilgili hesaplamalar

---

## Repository Access | Repository Erişimi

This repository is private and contains project-specific embedded software.

Bu repository gizlidir ve projeye özel gömülü sistem yazılımı içermektedir.

Source code and implementation details should not be redistributed without permission.

Kaynak kod ve uygulama detayları izin alınmadan yeniden paylaşılmamalıdır.

---

## Development Period | Geliştirme Dönemi

**2025**

Developed as part of the AZAK Rocket Team rocket avionics and payload development process.

AZAK Roket Takımı'nın roket aviyonik ve görev yükü geliştirme süreci kapsamında geliştirilmiştir.

---

## Team | Takım

**AZAK Rocket Team**

---

## Technologies

`Arduino` `C++` `FreeRTOS` `Embedded Systems` `Serial Communication` `Telemetry` `memcpy`
