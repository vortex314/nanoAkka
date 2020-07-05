#include <Cli.h>
#include <esp_log.h>

Cli::Cli(UART &uart) : _uart(uart) {}

void Cli::init() {
  INFO("CLI INIT");
  //  _uart = UART::create(0,1,3);
  _uart.setClock(115200);
  _uart.onRxd(onReceive, this);
  _uart.mode("8N1");
  _uart.init();
  for (int i = 0; i < CLI_MAX_LINES; i++)
    _lines[i] = "";
}

void Cli::writer(char *, uint32_t length) {} // to dev/null

void Cli::onToken(Token token, char ch) {
  switch (token) {
  case TK_CHAR: {
    if (_cursorIndex < line().length())
      insChar(ch);
    else
      addChar(ch);
    break;
  }
  case TK_ENTER: {
    onLine(line());
    writeCrLf();
    nextLine();
    _cursorIndex = line().length();
    writeLine();
    break;
  }
  case TK_BS: {
    backspace();
    break;
  }
  case TK_UP_ARROW: {
    prevLine();
    _cursorIndex = line().length();
    writeLine();
    break;
  }
  case TK_DOWN_ARROW: {
    nextLine();
    _cursorIndex = line().length();
    writeLine();
    break;
  }
  case TK_LEFT_ARROW: {
    if (leftCursor())
      write(CSI "1D"); // left
    break;
  }
  case TK_RIGHT_ARROW: {
    if (rightCursor())
      write(CSI "1C"); // right
    break;
  }
  case TK_HOME: {
    _cursorIndex = 0;
    setCursor();
    break;
  }
  case TK_END: {
    _cursorIndex = line().length();
    setCursor();
    break;
  }
  case TK_DEBUG: {
    printf("\r\n cursor : %d length:%d \r\n", _cursorIndex, line().length());
    break;
  }
  }
}

void Cli::onChar(char ch) {
  switch (_tokenState) {
  case TS_NEW: {
    switch (ch) {
    case '\b': {
      onToken(TK_BS, 0);
      break;
    }
    case '\r': {
      onToken(TK_ENTER, 0);
      break;
    }
    case '\n': {
      break;
    }
    case '\004': {
      enableLog();
      break;
    }
    case ESC: {
      _tokenState = TS_ESC;
      break;
    }
    case '#': {
      onToken(TK_DEBUG, 0);
      break;
    }
    case '\t': {
      break;
    }
    default: { onToken(TK_CHAR, ch); }
    }
    break;
  }
  case TS_ESC: {
    switch (ch) {
    case LEFT_BRACKET: {
      _tokenState = TS_ESC_LB;
      break;
    }
    default: { _tokenState = TS_NEW; }
    }
    break;
  }
  case TS_ESC_LB: {
    _tokenState = TS_NEW;
    switch (ch) {
    case 'A': {
      onToken(TK_UP_ARROW, 0);
      break;
    }
    case 'B': {
      onToken(TK_DOWN_ARROW, 0);
      break;
    }
    case 'C': {
      onToken(TK_RIGHT_ARROW, 0);
      break;
    }
    case 'D': {
      onToken(TK_LEFT_ARROW);
      break;
    }
    case 'F': {
      onToken(TK_END);
      break;
    }
    case 'H': {
      onToken(TK_HOME);
      break;
    }
    default: {}
    }
  }
  }
}

void Cli::onLine(std::string &line) {
  if (line.compare("+++") == 0)
    disableLog();
  if (line.compare("---") == 0)
    enableLog();
}

void Cli::disableLog() {
  INFO("info logging disabled");
  _logFunction = logger.writer();
  logger.writer(writer);
  esp_log_level_set("*", ESP_LOG_ERROR);
}

void Cli::enableLog() {
  logger.writer(_logFunction);
  esp_log_level_set("*", ESP_LOG_INFO);
  INFO("info logging enabled");
}

void Cli::onReceive(void *ptr) {
  Cli &me = *(Cli *)ptr;
  while (me._uart.hasData()) {
    me.onChar(me._uart.read());
  }
}

void Cli::addChar(char ch) {
  line() += ch;
  rightCursor();
  _uart.write(ch);
}

void Cli::insChar(char ch) {
  line().insert(_cursorIndex, 1, ch);
  write(line().substr(_cursorIndex).c_str());
  setCursor();
}

void Cli::backspace() {
  //  printf("\r\n cursor : %d \r\n", _cursorIndex);
  if (_cursorIndex > 0 && (_cursorIndex - 1) <= line().length() &&
      line().length()) {
    line().erase(_cursorIndex - 1, 1);
    leftCursor();
    write(CSI "1D"); // left
    write(CSI "0K"); // erase till EOL
    write(line().substr(_cursorIndex).c_str());
    setCursor();
  }
}

std::string &Cli::line() { return _lines[_lineIndex]; }

bool Cli::nextLine() {
  _lineIndex++;
  if (_lineIndex >= CLI_MAX_LINES)
    _lineIndex = 0;
  return true;
}

bool Cli::prevLine() {
  _lineIndex--;
  if (_lineIndex < 0) {
    _lineIndex = CLI_MAX_LINES - 1;
  }
  return true;
}

bool Cli::leftCursor() {
  _cursorIndex--;
  if (_cursorIndex < 0) {
    _cursorIndex = 0;
    return false;
  }
  return true;
}

bool Cli::rightCursor() {
  _cursorIndex++;
  return true;
}
#define PROMPT_LENGTH 8
void Cli::writeLine() {
  _uart.write('\r');
  write(CSI "7m");
  _uart.write('0' + _lineIndex);
  _uart.write(':');
  _uart.write('0' + line().length());
  write(" > " CSI "0m ");
  for (int i = 0; i < line().length(); i++) {
    _uart.write(line()[i]);
  }
  write(CSI "0K"); // erase till EOL
}

void Cli::write(const char *s) {
  int sLen = strlen(s);
  for (int i = 0; i < sLen; i++) {
    _uart.write(s[i]);
  }
}

void Cli::setCursor() {
  write(CSI);
  std::string pos;
  string_format(pos, "%d", _cursorIndex + PROMPT_LENGTH);
  write(pos.c_str());
  write("G");
}

void Cli::writeCrLf() { write("\r\n"); }