import rospy
from yapykaldi.grammar import Grammar
from grammar_parser.cfgparser import CFGParser


class CfgKaldiGrammar(Grammar):
    def __init__(self, cfg_parser, target):
        # type: (CFGParser, str, int) -> None
        super(self, CfgKaldiGrammar).__init__()
        self.parser = cfg_parser

        self.target = target

        self.recognized_sentence = []

    def traverse(self, recognised_word):
        # type: (str) -> bool
        """
        # TODO: expand the full tree, not only the first word
        """

        possible_next_words = self.parser.next_word(self.target, self.recognized_sentence)
        if recognised_word in possible_next_words:
            self.recognized_sentence += [recognised_word]
            return True
        else:
            return False