"use strict";

import React from 'react';

import ItemForm from './../components/ItemForm';

import ItemService from '../services/ItemService';
import CategoryService from '../services/CategoryService';

/*
 * assuming the API returns something like this:
 *   const json = [
 *      { value: 'one', label: 'One' },
 *      { value: 'two', label: 'Two' }
 *   ]
 */

export class ItemFormView extends React.Component {

    constructor(props) {
        super(props);
        this.state = {
            loading: true,
            item: undefined,
            error: undefined
        };
    }
    componentDidMount(){
        CategoryService.getCategories().then((data) => {
            this.setState({
                loading: false,
                data: [...data],
                item: undefined,
                error: undefined
            });
        }).catch((e) => {
            console.error(e);
        });
    }

    componentWillMount(){
        if(this.props.history.location.pathname == '/add') {
            this.setState({
                loading: true,
                data: [],
                item: undefined,
                error: undefined
            });
        }
        else if(this.props.location.state != undefined && this.props.location.state.item != undefined) {
            this.setState({
                loading: false,
                data: [],
                item: this.props.location.state.item,
                error: undefined
            });
        }
        else {
            this.setState({
                loading: true,
                data: [],
                error: undefined
            });

            let id = this.props.match.params.id;

            ItemService.getItem(id).then((data) => {
                this.setState({
                    item: data,
                    loading: false,
                    error: undefined
                });
            }).catch((e) => {
                console.error(e);
            });
        }
    }

    updateItem(item) {
        if(this.state.item == undefined) {
            ItemService.createItem(item).then((data) => {
                this.props.history.push('/my_items');
            }).catch((e) => {
                console.error(e);
                this.setState(Object.assign({}, this.state, {error: 'Error while creating item'}));
            });
        } else {
            ItemService.updateItem(item).then((data) => {
                this.props.history.goBack();
            }).catch((e) => {
                console.error(e);
                this.setState(Object.assign({}, this.state, {error: 'Error while creating movie'}));
            });
        }
    }

    render() {
        const  categories  = this.state.data;

        return (!this.state.loading && <ItemForm item={this.state.item} data = {categories} onSubmit={(item) => this.updateItem(item)} error={this.state.error} />);
    }
}
