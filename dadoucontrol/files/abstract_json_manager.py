import jsonpath_rw_ext

class AbstractJsonManager:
# '{}_{}_{}_{}'.format(s1, i, s2, f)

    def __init__(self, base_path):
        self.json_path = base_path + "json/"

    @staticmethod
    def standard_return(result, return_first, attribut):
        # logging.debug(result)
        to_return = {}
        error = False

        if not bool(result):
            error = True
        else:
            if return_first:
                if len(result) > 0:
                    to_return = result[0]
                else:
                    error = True
            else:
                to_return = result
            if attribut:
                to_return = to_return[attribut]

        if not error:
            return to_return

        return None

    @staticmethod
    def find(json_data, iterate_key, expression):
        result = 0
        for seq in json_data[iterate_key]:
            if len(jsonpath_rw_ext.match(expression, seq)) > 0:
                result = seq
        return result

    @staticmethod
    def get_attribut(json_object, key):
        if key in json_object:
            return json_object[key]
        else:
            return None

    def get_config(self) -> []:
        return self.config

    def get_first_level_attribut_match_key(self, json_file, first_level, match_key, match_value, attribut):
        result = jsonpath_rw_ext.match('$.{}[?{}~{}]'.format(first_level, match_key, match_value), json_file)
        return self.standard_return(result, True, attribut)

