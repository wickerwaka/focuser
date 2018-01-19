#define EEPROM_SIZE 1024

typedef unsigned short SaveData;

struct SaveBlock
{
  unsigned long sequence;
  SaveData data;
  unsigned short crc;
} __attribute__((packed, aligned(1)));
  
class SaveStateManager {
  public:

  private:
 
  unsigned short CalculateCRC(const SaveBlock* block) {
    unsigned char x;
    unsigned short crc = 0xFFFF;
    unsigned char length = sizeof(SaveData) + sizeof(unsigned long);
    unsigned char *ptr = (unsigned char *)block;
    while (length--){
      x = crc >> 8 ^ *ptr++;
      x ^= x>>4;
      crc = (crc << 8) ^ ((unsigned short)(x << 12)) ^ ((unsigned short)(x <<5)) ^ ((unsigned short)x);
    }
    return crc;
  }

  void UpdateCRC(SaveBlock* block) {
    block->crc = CalculateCRC(block);
  }

  bool ValidateCRC(const SaveBlock* block) {
    return block->crc == CalculateCRC(block);
  }  
};

