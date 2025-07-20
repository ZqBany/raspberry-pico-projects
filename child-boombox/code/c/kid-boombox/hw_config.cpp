#include <vector>
#include "sd_card.h"

#include "hw_config.h"

static std::vector<spi_t *> spis;             // SPI H/W components
static std::vector<sd_spi_if_t *> spi_ifs;    // SPI Interfaces
static std::vector<sd_card_t *> sd_cards;     // SD Card Sockets

size_t sd_get_num() { return sd_cards.size(); }
sd_card_t *sd_get_by_num(size_t num) {
    if (num <= sd_get_num()) {
        return sd_cards[num];
    } else {
        return NULL;
    }
}

size_t spi_get_num() { return spis.size(); }
spi_t *spi_get_by_num(size_t num) {
    if (num <= sd_get_num()) {
        return spis[num];
    } else {
        return NULL;
    }
}

size_t spi_if_get_num() { return spi_ifs.size(); }
sd_spi_if_t *spi_if_get_by_num(size_t num) {
    if (num <= spi_if_get_num()) {
        return spi_ifs[num];
    } else {
        return NULL;
    }
}

void add_spi(spi_t *spi) { spis.push_back(spi); }
void add_spi_if(sd_spi_if_t *spi_if) { spi_ifs.push_back(spi_if); }
void add_sd_card(sd_card_t *sd_card) { sd_cards.push_back(sd_card); }

/* [] END OF FILE */