#ifndef XTTS_OFFLINE_H
#define XTTS_OFFLINE_H

extern "C"
{

int text_to_speech(const char* src_text, const char* des_path, const char* param);
int login();
void logout();

}

#endif // XTTS_OFFLINE_H
