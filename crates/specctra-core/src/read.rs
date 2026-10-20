// SPDX-FileCopyrightText: 2024 Topola contributors
//
// SPDX-License-Identifier: MIT

use super::error::{ParseError, ParseErrorContext};
use super::structure::Parser;
use super::ListToken;

/// Reads one UTF-8-encoded `char` from `reader`, or `None` at EOF.
fn read_char(reader: &mut impl std::io::BufRead) -> std::io::Result<Option<char>> {
    let mut buf = [0u8; 4];
    let Some(&first) = reader.fill_buf()?.first() else {
        return Ok(None);
    };
    buf[0] = first;
    reader.consume(1);

    let len = match first {
        0x00..=0x7f => 1,
        0xc0..=0xdf => 2,
        0xe0..=0xef => 3,
        0xf0..=0xf7 => 4,
        _ => {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "invalid UTF-8 leading byte",
            ))
        }
    };
    for byte in &mut buf[1..len] {
        let Some(&next) = reader.fill_buf()?.first() else {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                "truncated UTF-8 sequence",
            ));
        };
        *byte = next;
        reader.consume(1);
    }
    Ok(std::str::from_utf8(&buf[..len])
        .map_err(|err| std::io::Error::new(std::io::ErrorKind::InvalidData, err))?
        .chars()
        .next())
}

pub struct InputToken {
    pub token: ListToken,
    pub context: (usize, usize),
}

impl InputToken {
    pub fn new(token: ListToken, context: (usize, usize)) -> Self {
        Self { token, context }
    }

    pub fn expect_any_start(self) -> Result<String, ParseErrorContext> {
        if let ListToken::Start { name } = self.token {
            Ok(name.to_ascii_lowercase())
        } else {
            Err(ParseError::ExpectedStartOfList("").add_context(self.context))
        }
    }

    pub fn expect_end(self) -> Result<(), ParseErrorContext> {
        if let ListToken::End = self.token {
            Ok(())
        } else {
            Err(ParseError::ExpectedEndOfList.add_context(self.context))
        }
    }
}

pub trait ReadDsn<R: std::io::BufRead>: Sized {
    fn read_dsn(tokenizer: &mut ListTokenizer<R>) -> Result<Self, ParseErrorContext>;
}
// custom impl feeding the read values back into the tokenizer
impl<R: std::io::BufRead> ReadDsn<R> for Parser {
    fn read_dsn(tokenizer: &mut ListTokenizer<R>) -> Result<Self, ParseErrorContext> {
        Ok(Self {
            string_quote: tokenizer
                .read_optional(&["string_quote"])?
                .inspect(|v| tokenizer.quote_char = Some(*v)),
            space_in_quoted_tokens: tokenizer
                .read_optional(&["space_in_quoted_tokens"])?
                .inspect(|v| tokenizer.space_in_quoted = *v),
            // Some exporters (EasyEDA) quote these values without
            // declaring `string_quote`, so the tokenizer splits them on
            // interior spaces. Read all leaves and rejoin — a lone
            // outer quote pair is the undeclared quoting, strip it.
            host_cad: tokenizer
                .read_optional_leaves("host_cad")?
                .map(rejoin_undeclared_quotes),
            host_version: tokenizer
                .read_optional_leaves("host_version")?
                .map(rejoin_undeclared_quotes),
        })
    }
}

fn rejoin_undeclared_quotes(leaves: Vec<String>) -> String {
    let joined = leaves.join(" ");
    joined
        .strip_prefix('"')
        .and_then(|s| s.strip_suffix('"'))
        .map(str::to_string)
        .unwrap_or(joined)
}

impl<R: std::io::BufRead> ReadDsn<R> for String {
    fn read_dsn(tokenizer: &mut ListTokenizer<R>) -> Result<Self, ParseErrorContext> {
        let inptoken = tokenizer.consume_token()?;
        if let ListToken::Leaf { value } = inptoken.token {
            Ok(value)
        } else {
            Err(ParseError::ExpectedLeaf.add_context(inptoken.context))
        }
    }
}

/// Optional anonymous leaf — e.g. `Class::name`, which some exporters
/// (freerouting, EasyEDA) omit entirely on anonymous classes.
impl<R: std::io::BufRead> ReadDsn<R> for Option<String> {
    fn read_dsn(tokenizer: &mut ListTokenizer<R>) -> Result<Self, ParseErrorContext> {
        let input = tokenizer.consume_token()?;
        Ok(if let ListToken::Leaf { .. } = input.token {
            tokenizer.return_token(input);
            Some(tokenizer.read_value::<String>()?)
        } else {
            tokenizer.return_token(input);
            None
        })
    }
}

impl<R: std::io::BufRead> ReadDsn<R> for char {
    fn read_dsn(tokenizer: &mut ListTokenizer<R>) -> Result<Self, ParseErrorContext> {
        let err = tokenizer.add_context(ParseError::Expected("a single character"));
        let string = String::read_dsn(tokenizer)?;
        let mut it = string.chars();
        let first = match it.next() {
            None => return Err(err),
            Some(x) => x,
        };
        match it.next() {
            None => Ok(first),
            Some(_) => Err(err),
        }
    }
}

impl<R: std::io::BufRead> ReadDsn<R> for bool {
    fn read_dsn(tokenizer: &mut ListTokenizer<R>) -> Result<Self, ParseErrorContext> {
        match String::read_dsn(tokenizer)?.as_str() {
            "on" => Ok(true),
            "off" => Ok(false),
            _ => Err(tokenizer.add_context(ParseError::Expected("boolean"))),
        }
    }
}

/// `impl_ReadDsn_via_FromStr!((TYPE, TYPE-NAME); ...)`
macro_rules! impl_ReadDsn_via_FromStr {
    ($(($t:ty, $name:expr));* $(;)?) => {
        $( impl<R: std::io::BufRead> ReadDsn<R> for $t {
            fn read_dsn(tokenizer: &mut ListTokenizer<R>) -> Result<Self, ParseErrorContext> {
                String::read_dsn(tokenizer)?
                    .parse()
                    .map_err(|_| tokenizer.add_context(ParseError::Expected($name)))
            }
        } )*
    }
}

impl_ReadDsn_via_FromStr!(
    (i32, "i32");
    (u32, "u32");
    (usize, "usize");
    (f32, "f32");
    (f64, "f64");
);

pub struct ListTokenizer<R> {
    reader: R,
    peeked_char: Option<char>,
    cached_token: Option<InputToken>,
    space_in_quoted: bool,
    quote_char: Option<char>,
    line: usize,
    column: usize,
}

impl<R> ListTokenizer<R> {
    pub fn new(reader: R) -> Self {
        Self {
            reader,
            peeked_char: None,
            cached_token: None,
            space_in_quoted: false,
            quote_char: None,
            line: 1,
            column: 0,
        }
    }

    pub fn context(&self) -> (usize, usize) {
        (self.line, self.column)
    }

    pub(crate) fn add_context(&self, error: ParseError) -> ParseErrorContext {
        ParseErrorContext {
            error,
            context: (self.line, self.column),
        }
    }
}

impl<R: std::io::BufRead> ListTokenizer<R> {
    fn next_char(&mut self) -> Result<char, ParseErrorContext> {
        let return_chr = self.peek_char()?;
        self.reset_char();
        Ok(return_chr)
    }

    /// discard last peeked character, move cursor forward
    fn reset_char(&mut self) -> Option<char> {
        let ret = self.peeked_char.take();
        if let Some(return_chr) = ret {
            if return_chr == '\n' {
                self.line += 1;
                self.column = 0;
            } else {
                self.column += 1;
            }
        }
        ret
    }

    fn peek_char(&mut self) -> Result<char, ParseErrorContext> {
        Ok(if let Some(chr) = self.peeked_char {
            chr
        } else {
            let chr = read_char(&mut self.reader)
                .transpose()
                .ok_or(self.add_context(ParseError::Eof))?
                .map_err(|err| self.add_context(err.into()))?;
            self.peeked_char = Some(chr);
            chr
        })
    }

    fn skip_whitespace(&mut self) -> Result<(), ParseErrorContext> {
        loop {
            let chr = self.peek_char()?;
            if chr == ' ' || chr == '\r' || chr == '\n' {
                self.reset_char();
            } else {
                return Ok(());
            }
        }
    }

    fn read_string(&mut self) -> Result<String, ParseErrorContext> {
        fn read_quoted<R: std::io::BufRead>(
            this: &mut ListTokenizer<R>,
            quote_chr: char,
        ) -> Result<String, ParseErrorContext> {
            let mut string = String::new();
            this.reset_char();

            loop {
                let ctx = this.context();
                let chr = this.next_char()?;
                if chr == ' ' && !this.space_in_quoted {
                    return Err(ParseError::UnexpectedSpaceInQuotedStr.add_context(ctx));
                } else if chr == quote_chr {
                    break;
                } else {
                    string.push(chr);
                }
            }

            Ok(string)
        }

        if let Some(quote_chr) = self.quote_char {
            if quote_chr == self.peek_char()? {
                return read_quoted(self, quote_chr);
            }
        }
        self.read_unquoted()
    }

    fn read_unquoted(&mut self) -> Result<String, ParseErrorContext> {
        let mut string = String::new();

        loop {
            let chr = self.peek_char()?;
            if chr == ' ' || chr == '(' || chr == ')' || chr == '\r' || chr == '\n' {
                break;
            }
            string.push(chr);
            self.reset_char();
        }

        if string.is_empty() {
            Err(self.add_context(ParseError::Expected("string (unquoted)")))
        } else {
            Ok(string)
        }
    }

    // the following two methods effectively allow 1 token of lookahead

    // returns next token, either a cached one returned earlier or a newly read one
    pub fn consume_token(&mut self) -> Result<InputToken, ParseErrorContext> {
        // move out of cache if not empty, otherwise consume input
        // always leaves cache empty
        Ok(if let Some(token) = self.cached_token.take() {
            token
        } else {
            self.read_token()?
        })
    }

    // puts a token back into cache, to be consumed by something else
    pub fn return_token(&mut self, token: InputToken) {
        assert!(self.cached_token.is_none());
        self.cached_token = Some(token);
    }

    fn read_token(&mut self) -> Result<InputToken, ParseErrorContext> {
        self.skip_whitespace()?;
        let context = self.context();

        let chr = self.peek_char()?;
        Ok(InputToken::new(
            if chr == '(' {
                self.reset_char();
                self.skip_whitespace()?;
                ListToken::Start {
                    name: self.read_string()?,
                }
            } else if chr == ')' {
                self.reset_char();
                ListToken::End
            } else {
                ListToken::Leaf {
                    value: self.read_string()?,
                }
            },
            context,
        ))
    }

    pub fn read_value<T: ReadDsn<R>>(&mut self) -> Result<T, ParseErrorContext> {
        T::read_dsn(self)
    }

    pub fn read_named<T: ReadDsn<R>>(
        &mut self,
        valid_names: &[&'static str],
    ) -> Result<T, ParseErrorContext> {
        assert!(!valid_names.is_empty());
        self.read_optional(valid_names)?
            .ok_or_else(|| self.add_context(ParseError::ExpectedStartOfList(valid_names[0])))
    }

    pub fn read_optional<T: ReadDsn<R>>(
        &mut self,
        valid_names: &[&'static str],
    ) -> Result<Option<T>, ParseErrorContext> {
        let input = self.consume_token()?;
        Ok(if input.token.is_start_of(valid_names) {
            let value = self.read_value::<T>()?;
            self.consume_token()?.expect_end()?;
            Some(value)
        } else {
            self.return_token(input);
            None
        })
    }

    /// Reads an optional `(name leaf*)` list, returning the raw leaf
    /// values. Unlike `read_optional` this tolerates values the lexer
    /// split into several leaves, e.g. quoted strings in files that
    /// never declared `string_quote`.
    fn read_optional_leaves(
        &mut self,
        name: &'static str,
    ) -> Result<Option<Vec<String>>, ParseErrorContext> {
        let input = self.consume_token()?;
        Ok(if input.token.is_start_of(&[name]) {
            let leaves = self.read_array::<String>()?;
            self.consume_token()?.expect_end()?;
            Some(leaves)
        } else {
            self.return_token(input);
            None
        })
    }

    pub fn read_array<T: ReadDsn<R>>(&mut self) -> Result<Vec<T>, ParseErrorContext> {
        let mut array = Vec::<T>::new();
        loop {
            let input = self.consume_token()?;
            if let ListToken::Leaf { .. } = input.token {
                self.return_token(input);
                array.push(self.read_value::<T>()?);
            } else {
                self.return_token(input);
                break;
            }
        }
        Ok(array)
    }

    pub fn read_named_array<T: ReadDsn<R>>(
        &mut self,
        valid_names: &[&'static str],
    ) -> Result<Vec<T>, ParseErrorContext> {
        let mut array = Vec::new();
        while let Some(value) = self.read_optional::<T>(valid_names)? {
            array.push(value);
        }
        Ok(array)
    }
}
