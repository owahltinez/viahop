//! Specctra S-expression lexer, parser, and formatter.

use std::fmt;

/// A node in a Specctra S-expression tree.
#[derive(Debug, Clone, PartialEq)]
pub enum SExp {
    Atom(String),
    List(Vec<SExp>),
}

impl SExp {
    pub fn as_atom(&self) -> Option<&str> {
        match self {
            Self::Atom(s) => Some(s.as_str()),
            Self::List(_) => None,
        }
    }

    pub fn as_list(&self) -> Option<&[SExp]> {
        match self {
            Self::List(items) => Some(items.as_slice()),
            Self::Atom(_) => None,
        }
    }

    pub fn as_f64(&self) -> Option<f64> {
        self.as_atom()?.parse::<f64>().ok()
    }

    pub fn as_i64(&self) -> Option<i64> {
        self.as_atom()?.parse::<i64>().ok()
    }

    /// Leading atom of a list node (e.g., `"layer"` for `(layer F.Cu ...)`).
    pub fn tag(&self) -> Option<&str> {
        self.as_list()?.first()?.as_atom()
    }

    /// Returns the first child list whose tag matches `tag`.
    pub fn find_child(&self, tag: &str) -> Option<&SExp> {
        self.as_list()?
            .iter()
            .skip(1)
            .find(|c| c.tag() == Some(tag))
    }

    /// Iterates over all direct child lists whose tag matches `tag`.
    pub fn children_with_tag<'a>(&'a self, tag: &'a str) -> impl Iterator<Item = &'a SExp> {
        let slice = self.as_list().unwrap_or(&[]);
        slice
            .iter()
            .skip(1)
            .filter(move |c| c.tag() == Some(tag))
    }
}

/// Parses a single top-level Specctra S-expression.
pub fn parse(input: &str) -> Result<SExp, String> {
    let quote_char = detect_string_quote(input).unwrap_or('"');
    let mut parser = Parser {
        bytes: input.as_bytes(),
        pos: 0,
        quote: quote_char as u8,
    };
    parser.skip_ws();
    let expr = parser.parse_expr()?;
    parser.skip_ws();
    Ok(expr)
}

/// Parses a top-level Specctra `(pcb ...)` document, tolerating stray extra `)`
/// tokens that prematurely close `(pcb ...)` before sibling sections such as
/// `(placement ...)`, `(library ...)`, `(network ...)`, or `(wiring ...)`.
pub fn parse_dsn_tolerant(input: &str) -> Result<SExp, String> {
    let quote_char = detect_string_quote(input).unwrap_or('"');
    let mut parser = Parser {
        bytes: input.as_bytes(),
        pos: 0,
        quote: quote_char as u8,
    };
    parser.skip_ws();
    let mut root = parser.parse_expr()?;
    let mut extras = Vec::new();
    loop {
        parser.skip_ws();
        if parser.pos >= parser.bytes.len() {
            break;
        }
        // Skip stray closing parens at top level left by malformed DSN scripts.
        if parser.bytes[parser.pos] == b')' {
            parser.pos += 1;
            continue;
        }
        extras.push(parser.parse_expr()?);
    }
    if !extras.is_empty()
        && let SExp::List(ref mut items) = root {
            items.extend(extras);
        }
    Ok(root)
}

/// Parses zero or more top-level Specctra S-expressions (e.g., `.rules` files).
pub fn parse_all(input: &str) -> Result<Vec<SExp>, String> {
    let quote_char = detect_string_quote(input).unwrap_or('"');
    let mut parser = Parser {
        bytes: input.as_bytes(),
        pos: 0,
        quote: quote_char as u8,
    };
    let mut out = Vec::new();
    loop {
        parser.skip_ws();
        if parser.pos >= parser.bytes.len() {
            break;
        }
        out.push(parser.parse_expr()?);
    }
    Ok(out)
}

/// Formats an atom for Specctra output, quoting it when needed.
pub fn format_atom(s: &str) -> String {
    if s.is_empty() {
        return "\"\"".to_string();
    }
    let needs_quote = s.bytes().any(|b| {
        b.is_ascii_whitespace()
            || matches!(b, b'(' | b')' | b'"' | b'[' | b']' | b':' | b';' | b'#')
    });
    if !needs_quote {
        return s.to_string();
    }
    let mut out = String::with_capacity(s.len() + 2);
    out.push('"');
    for ch in s.chars() {
        if ch == '"' {
            out.push('\\');
        }
        out.push(ch);
    }
    out.push('"');
    out
}

impl fmt::Display for SExp {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Atom(s) => f.write_str(&format_atom(s)),
            Self::List(items) => {
                f.write_str("(")?;
                for (i, item) in items.iter().enumerate() {
                    if i > 0 {
                        f.write_str(" ")?;
                    }
                    write!(f, "{item}")?;
                }
                f.write_str(")")
            }
        }
    }
}

fn detect_string_quote(input: &str) -> Option<char> {
    let idx = input.find("(string_quote")?;
    let rest = &input[idx + "(string_quote".len()..];
    for ch in rest.chars() {
        if ch.is_ascii_whitespace() {
            continue;
        }
        if ch == ')' {
            return None;
        }
        return Some(ch);
    }
    None
}

struct Parser<'a> {
    bytes: &'a [u8],
    pos: usize,
    quote: u8,
}

impl<'a> Parser<'a> {
    fn skip_ws(&mut self) {
        while self.pos < self.bytes.len() {
            let b = self.bytes[self.pos];
            if b.is_ascii_whitespace() {
                self.pos += 1;
            } else if b == b'#' {
                while self.pos < self.bytes.len() && self.bytes[self.pos] != b'\n' {
                    self.pos += 1;
                }
            } else {
                break;
            }
        }
    }

    fn parse_expr(&mut self) -> Result<SExp, String> {
        self.skip_ws();
        if self.pos >= self.bytes.len() {
            return Err("unexpected end of input".to_string());
        }
        match self.bytes[self.pos] {
            b'(' => self.parse_list(),
            b')' => Err(format!("unexpected ')' at byte {}", self.pos)),
            b if b == self.quote => self.parse_quoted(),
            _ => self.parse_atom(),
        }
    }

    fn parse_list(&mut self) -> Result<SExp, String> {
        // Explicit stack avoids recursion limit on deeply nested files.
        self.pos += 1;
        let mut stack: Vec<Vec<SExp>> = vec![Vec::new()];
        loop {
            self.skip_ws();
            if self.pos >= self.bytes.len() {
                return Err("unclosed '(' in S-expression".to_string());
            }
            let b = self.bytes[self.pos];
            if b == b'(' {
                self.pos += 1;
                stack.push(Vec::new());
            } else if b == b')' {
                self.pos += 1;
                let completed = stack.pop().unwrap();
                let node = SExp::List(completed);
                if let Some(parent) = stack.last_mut() {
                    parent.push(node);
                } else {
                    return Ok(node);
                }
            } else if b == self.quote {
                // Specctra `(string_quote ")` uses an unquoted literal quote char right before ')'.
                let is_string_quote_arg = stack
                    .last()
                    .and_then(|cur| cur.first())
                    .and_then(SExp::as_atom)
                    == Some("string_quote")
                    && stack.last().is_some_and(|cur| cur.len() == 1);
                if is_string_quote_arg {
                    self.pos += 1;
                    stack
                        .last_mut()
                        .unwrap()
                        .push(SExp::Atom((self.quote as char).to_string()));
                } else {
                    let atom = self.parse_quoted()?;
                    stack.last_mut().unwrap().push(atom);
                }
            } else {
                let atom = self.parse_atom()?;
                stack.last_mut().unwrap().push(atom);
            }
        }
    }

    fn parse_quoted(&mut self) -> Result<SExp, String> {
        let start = self.pos;
        self.pos += 1;
        let mut buf = Vec::new();
        while self.pos < self.bytes.len() {
            let b = self.bytes[self.pos];
            if b == b'\\' && self.pos + 1 < self.bytes.len() && self.bytes[self.pos + 1] == self.quote {
                buf.push(self.quote);
                self.pos += 2;
            } else if b == self.quote {
                self.pos += 1;
                let s = String::from_utf8(buf)
                    .map_err(|e| format!("invalid UTF-8 at byte {start}: {e}"))?;
                return Ok(SExp::Atom(s));
            } else {
                buf.push(b);
                self.pos += 1;
            }
        }
        Err(format!("unterminated quoted string starting at byte {start}"))
    }

    fn parse_atom(&mut self) -> Result<SExp, String> {
        let start = self.pos;
        while self.pos < self.bytes.len() {
            let b = self.bytes[self.pos];
            if b.is_ascii_whitespace() || b == b'(' || b == b')' {
                break;
            }
            self.pos += 1;
        }
        let s = std::str::from_utf8(&self.bytes[start..self.pos])
            .map_err(|e| format!("invalid UTF-8 at byte {start}: {e}"))?;
        Ok(SExp::Atom(s.to_string()))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn parses_nested_and_quoted() {
        let src = r#"(pcb "/tmp/a b.dsn" (parser (string_quote ")) (net /Audio/CLAMP_L (pins D1-3)))"#;
        let expr = parse(src).unwrap();
        assert_eq!(expr.tag(), Some("pcb"));
        assert_eq!(expr.as_list().unwrap()[1].as_atom(), Some("/tmp/a b.dsn"));
        let net = expr.find_child("net").unwrap();
        assert_eq!(net.as_list().unwrap()[1].as_atom(), Some("/Audio/CLAMP_L"));
    }

    #[test]
    fn rejects_unclosed_paren() {
        assert!(parse("(pcb (structure)").is_err());
    }
}
