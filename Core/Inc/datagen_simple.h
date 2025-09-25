#ifndef DATAGEN_SIMPLE_H
#define DATAGEN_SIMPLE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// Máscara de modos do simulador
#define SIM_MODE_NONE       0u
#define SIM_MODE_CAR        (1u << 0)  // simula dados do "carro"
#define SIM_MODE_CONTROLS   (1u << 1)  // simula volante/velocidade/direção

// Inicializa o simulador.
// - mode_mask: combine SIM_MODE_CAR | SIM_MODE_CONTROLS para simular tudo
// - who_tag: identificador de origem ("central", etc.)
void DataGenSimple_Init(uint32_t mode_mask, const char *who_tag);

// Monta um JSON completo no buffer 'out' (capacidade 'cap').
// - now_ms: timestamp (ms) para registrar no JSON
// - controls_override_json: se NÃO for NULL, será embutido como bloco "controls"
//   Caso seja NULL e SIM_MODE_CONTROLS estiver ativo, o simulador gera "controls".
//
// Retorna o tamanho escrito (>=0) ou 0/negativo em erro.
size_t DataGenSimple_BuildFullJSON(char *out, size_t cap, uint32_t now_ms,
                                   const char *controls_override_json);

#ifdef __cplusplus
}
#endif

#endif // DATAGEN_SIMPLE_H
