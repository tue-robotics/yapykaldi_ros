import rospy
from yapykaldi.grammar import Grammar, Rule
from grammar_parser.cfgparser import CFGParser

from openfst_python import Fst, Arc, Weight


class CfgKaldiGrammar(Grammar):
    def __init__(self, cfg_parser, target):
        # type: (CFGParser, str, int) -> None
        super(self, CfgKaldiGrammar).__init__()
        self._parser = cfg_parser

        self._target = target

        self._get_words()
        self._tree_root = self._expand_tree()

    def as_finite_state_transducer(self):
        f = Fst()
        nodes = {node: f.add_state() for node in SentenceNode.instances}
        for node in SentenceNode.instances:
            for edge in node.edges:
                f.add_arc(nodes[node],
                          Arc(1,
                              2,
                              Weight(f.weight_type(), 1.0),
                              nodes[edge.node]))

        # Root?
        f.set_start(nodes[SentenceNode.instances[0]])
        # Must be deepest node or with most distance from start?
        f.set_final(nodes[SentenceNode.instances[-1]])

        return f

    def _get_words(self):
        # type: () -> List[str]
        """
        Extracts list with all the unique words, used within the grammar
        """

        # Extract rules from the grammar file
        rules = self._parser.rules

        # Extract words
        words = set()

        for key, value in rules.iteritems():
            # Get the list of options for the rule value
            options = value.options
            for option in options:
                # Get the list of conjuncts for option 'option'
                conjuncts = option.conjuncts
                for conjunct in conjuncts:
                    # If conjunct is not a variable put its value in the set of words
                    if not conjunct.is_variable:
                        words.add(conjunct.name)

        words = [word.upper() for word in list(words)]
        words.sort()

        return words

    def _expand_tree(self):
        """
        Expands the grammar tree based on the words in the grammar rules for the
        pre-set target

        :return: tree of sentence nodes
        """
        # Extract rules from the grammar file
        rules = self._parser.rules
        return expand_tree(rules, self._target)


class SentenceNode:
    """
    A node in a sentence.
    :ivar edges: Edges to the next node.
    :ivar done: Reached the end of the sentence.
    """
    instances = []

    def __init__(self):
        self.edges = []
        self.done = False

        SentenceNode.instances += [self]


class SentenceEdge:
    """
    An edge in a sentence.
    :ivar word: The word to be understood.
    :ivar node: Node for the remainder of the sentence.
    """
    instances = []

    def __init__(self, word, node):
        self.word = word
        self.node = node

        SentenceEdge.instances += [self]


def expand_tree(rules, target='T'):
    # type: (List[Rule], str) -> SentenceNode
    """
    Expands the grammar tree based on the words in the grammar rules.

    :param rules: Extracted rules from the grammar file.
    :param target: Target rule to expand, default is 'T'.
    :return: The root of the expanded tree.
    :rtype: SentenceNode
    """
    # Map of set of successor rules to nodes.
    available_nodes = {}

    # Pairs of node and rule suffixes that need further work.
    work_list = []

    # Construct the initial node and the first set of suffix rules to expand further.
    root_list = [opt.conjuncts[:] for opt in rules[target].options]
    root_node = assign_node(root_list, available_nodes, work_list, rules)
    while work_list:
        node, expanded_list = work_list.pop()

        # collects alternatives on common prefixes and stores successor sentences
        prefix_dict = {}
        for item in expanded_list:
            successors = prefix_dict.get(item[0].name)
            if successors:
                # Store the expanded successor sentence in existing entry.
                successors.append(item[1:])
            else:
                # Store the expanded successor sentence found a non-existing prefix.
                prefix_dict[item[0].name] = [item[1:]]

        # Iterate over the collected prefixes and make a new edge for the words.
        for word, successors in prefix_dict.items():
            # Find the node to jump to after recognizing 'word'.
            nextnode = assign_node(successors, available_nodes, work_list, rules)
            edge = SentenceEdge(word, nextnode)
            node.edges.append(edge)

    return root_node


def assign_node(sentence_list, available_nodes, work_list, rules):
    # type: (List[str], Mapping[str, SentenceNode], List[Tuple[SentenceNode, Rule, List[str]]], List[Rule]) -> SentenceNode
    """
    For a given list of rule suffixes, find or add a node, and update the work list if necessary.

    :param sentence_list: List of rule suffixes to find or add a node for.
    :type  sentence_list: List of rule alternatives (a list of conjuncts, partly expanded to words,
            in particular, the first conjuct shou d not be a variable).

    :param available_nodes: Known set of rule sufixes and their associated nodes. May be updated.
    :type  available_nodes: Dict of str to SentenceNode

    :param work_list: List or rule suffixes that need further processing. May be updated.
    :type  work_list: List of pairs (node, rule suffixes).

    :param rules: Rules of the grammar.

    :return: Node associated with the provided sentence_list.
    """
    end_found, sentence_list = expand_sentences(sentence_list, rules)
    sentence_set = stringify_suffixes(sentence_list)
    sentence_set = frozenset(sentence_set)
    node = available_nodes.get(sentence_set)
    if node is None:
        node = SentenceNode()
        node.done = end_found
        available_nodes[sentence_set] = node

        non_empty_sentences = []
        for sentence in sentence_list:
            if sentence:
                non_empty_sentences.append(sentence)
            else:
                node.done = True

        work_list.append((node, non_empty_sentences))
    return node


def expand_sentences(sentence_list, rules):
    # type: (List[str], List[Rule]) -> Tuple[bool, List[str]]
    """
    Expands the grammar rules until elimination of all variables at the first position

    :param sentence_list: List of grammar rules
    :param rules: Rules of the grammar
    :return: Expanded list, an whether an end of an sentence was found.
    """
    end_found = False
    while sentence_list:
        # decide if we need to expand anything
        not_expanded = False
        for item in sentence_list:
            # Need to remove all empty alternatives.
            if not item:
                not_expanded = True
                end_found = True
                continue

            # Found an alternative, that needs further expansion.
            if item[0].is_variable:
                not_expanded = True

        # All first enries are words already, done!
        if not not_expanded:
            break

        # Expand variables at the first entry.
        expanded_list = []
        for item in sentence_list:
            if not item:
                continue
            if not item[0].is_variable:
                expanded_list.append(item)
                continue

            for opt in rules[item[0].name].options:
                d = opt.conjuncts + item[1:]
                expanded_list.append(d)

        sentence_list = expanded_list

    return end_found, sentence_list


def stringify_suffixes(expanded_list):
    # type: (List[str]) -> Set[str]
    """
    Convert the current rule suffixes to string form.

    :param expanded_list: List of rule suffixes to convert.
    :return: Set of suffixes, after converting each to a string.
    """
    sentence_set = set()
    for sentence in expanded_list:
        sentence_text = " ".join(conjunct.name for conjunct in sentence)
        sentence_set.add(sentence_text)
    return sentence_set