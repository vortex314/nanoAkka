#include <Hardware.h>
#include <NanoAkka.h>

#define ESC 0x1B
#define LEFT_BRACKET '['
#define CLI_MAX_LINES 5
#define CSI "\033["

class Cli {
  UART &_uart;
  std::string _lines[CLI_MAX_LINES];
  int _lineIndex = 0;
  int _cursorIndex = 0;

  std::string _rxd;
  LogFunction _logFunction;
  typedef enum {
    TK_CHAR,
    TK_BS,
    TK_ENTER,
    TK_UP_ARROW,
    TK_DOWN_ARROW,
    TK_LEFT_ARROW,
    TK_RIGHT_ARROW,
    TK_HOME,
    TK_END,
    TK_DEBUG
  } Token;
  typedef enum { TS_NEW, TS_ESC, TS_ESC_LB } TokenState;
  TokenState _tokenState = TS_NEW;

public:
  Cli(UART &uart);
  void init();
  static void onReceive(void *);
  void onChar(char ch);
  void onToken(Token toke, char ch = 0);
  void onLine(std::string &line);
  void backspace();
  void addChar(char ch);
  void insChar(char);
  void eraseChar(int idx);
  void disableLog();
  void enableLog();
  void writeLine();
  void writeCrLf();
  bool nextLine();
  bool prevLine();
  bool rightCursor();
  bool leftCursor();
  void setCursor();
  static void writer(char *s, uint32_t length);
  void write(const char *);
  std::string &line();
};